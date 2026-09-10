"""AMR BLE Wi-Fiプロビジョニング GATTサーバー本体。

Wi-Fi未接続時のみ `AMR-Setup-XXXX` としてアドバタイズし、Wi-Fi設定用
キャラクタリスティックへのWriteをトリガーにnmcli経由で接続、結果を
状態通知キャラクタリスティックへNotifyで返す。
"""

import json
import logging
import threading

from gi.repository import GLib

from bluezero import adapter
from bluezero import async_tools
from bluezero import peripheral

import wifi_manager

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("gatt_server")

SERVICE_UUID = "00000001-710e-4a5b-8804-000000000000"
WIFI_CONFIG_CHAR_UUID = "00000002-710e-4a5b-8804-000000000000"
STATUS_CHAR_UUID = "00000003-710e-4a5b-8804-000000000000"

WIFI_POLL_INTERVAL_S = 5

_status_char = None  # Notify購読中の状態通知キャラクタリスティック(未購読ならNone)
_advertising = False


def _local_name(adapter_address: str) -> str:
    suffix = adapter_address.replace(":", "")[-4:].upper()
    return f"AMR-Setup-{suffix}"


def _push_status(status: dict) -> bool:
    if _status_char is not None:
        _status_char.set_value(list(json.dumps(status).encode("utf-8")))
    return False  # GLib.idle_addは一度きり実行させる


def _on_status_notify(notifying, characteristic):
    # BlueZがCCCDを自動管理し、購読/解除のたびにこのcallbackを呼ぶ。
    # 参照を保持しておかないとset_value()でNotifyする対象が分からない。
    global _status_char
    _status_char = characteristic if notifying else None


def _connect_worker(ssid, password):
    status, detail = wifi_manager.connect(ssid, password)
    if status == "connected":
        GLib.idle_add(_push_status, {"status": "connected", "ip": detail})
    else:
        GLib.idle_add(_push_status, {"status": "failed", "error": detail})


def _on_wifi_config_write(value, _options):
    try:
        payload = json.loads(bytes(value).decode("utf-8"))
        ssid = payload["ssid"]
        password = payload["password"]
    except (ValueError, KeyError, UnicodeDecodeError) as exc:
        logger.warning("invalid provisioning payload: %s", exc)
        GLib.idle_add(_push_status, {"status": "failed", "error": "invalid_request"})
        return

    # nmcliの接続試行は最大60秒ブロックしうるため、GLibメインループ
    # (D-Bus/BLEの処理スレッド)を止めないよう別スレッドで実行する。
    GLib.idle_add(_push_status, {"status": "connecting"})
    threading.Thread(target=_connect_worker, args=(ssid, password), daemon=True).start()


def _poll_wifi_state(periph):
    global _advertising
    connected = wifi_manager.is_connected()
    if connected and _advertising:
        periph.ad_manager.unregister_advertisement(periph.advert)
        _advertising = False
        logger.info("wifi connected -> advertise stopped")
    elif not connected and not _advertising:
        periph.ad_manager.register_advertisement(periph.advert, {})
        _advertising = True
        logger.info("wifi disconnected -> advertise started")
    return True  # GLib.timeout_add_secondsを継続


def _run(periph, start_advertising):
    global _advertising
    for service in periph.services:
        periph.app.add_managed_object(service)
    for characteristic in periph.characteristics:
        periph.app.add_managed_object(characteristic)

    periph.advert.service_UUIDs = periph.primary_services
    if periph.local_name:
        periph.advert.local_name = periph.local_name

    if not periph.dongle.powered:
        periph.dongle.powered = True
    periph.srv_mng.register_application(periph.app, {})

    if start_advertising:
        periph.ad_manager.register_advertisement(periph.advert, {})
        _advertising = True

    async_tools.add_timer_seconds(WIFI_POLL_INTERVAL_S, _poll_wifi_state, periph)

    try:
        periph.mainloop.run()
    except KeyboardInterrupt:
        periph.mainloop.quit()
        if _advertising:
            periph.ad_manager.unregister_advertisement(periph.advert)


def main():
    dongle_address = list(adapter.Adapter.available())[0].address
    local_name = _local_name(dongle_address)

    periph = peripheral.Peripheral(dongle_address, local_name=local_name)
    periph.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)
    periph.add_characteristic(
        srv_id=1, chr_id=1, uuid=WIFI_CONFIG_CHAR_UUID,
        value=[], notifying=False,
        # encrypt-write: ペアリング/ボンディング済みリンクでの書込みのみ許可。
        # パスワードを平文JSONで送るため、暗号化されていないリンクでは書込ませない。
        flags=["write", "encrypt-write"],
        write_callback=_on_wifi_config_write,
    )
    periph.add_characteristic(
        srv_id=1, chr_id=2, uuid=STATUS_CHAR_UUID,
        value=[], notifying=False,
        flags=["read", "notify"],
        notify_callback=_on_status_notify,
    )

    logger.info("advertising as %s", local_name)
    _run(periph, start_advertising=not wifi_manager.is_connected())


if __name__ == "__main__":
    main()
