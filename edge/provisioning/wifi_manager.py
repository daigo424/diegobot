"""nmcliをsubprocessで叩き、Wi-Fi接続とIP取得・状態確認を行う薄いラッパー。"""

import subprocess

WLAN_IFACE = "wlan0"
CONNECT_TIMEOUT_S = 60

# nmcliの終了コードは仕様上 3=タイムアウト、4=接続アクティベーション失敗
# (パスワード誤りもここに含まれる)で固定されている。呼び出し側が返す
# エラー種別はauth_error/timeoutの2値のみなので、4以外の非0終了も
# すべてauth_errorへ丸める。
NMCLI_EXIT_TIMEOUT = 3


def connect(ssid: str, password: str) -> tuple[str, str]:
    """Wi-Fiへ接続を試みる。

    戻り値: (status, detail)
      status == "connected" のとき detail は割り当てられたIPv4アドレス
      status == "failed" のとき detail は "auth_error" または "timeout"
    """
    result = subprocess.run(
        [
            "nmcli", "-w", str(CONNECT_TIMEOUT_S),
            "device", "wifi", "connect", ssid,
            "password", password,
            "ifname", WLAN_IFACE,
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return "connected", _get_ip(WLAN_IFACE)
    if result.returncode == NMCLI_EXIT_TIMEOUT:
        return "failed", "timeout"
    return "failed", "auth_error"


def _get_ip(iface: str) -> str:
    result = subprocess.run(
        ["nmcli", "-g", "IP4.ADDRESS", "device", "show", iface],
        capture_output=True,
        text=True,
        check=True,
    )
    # "192.168.11.50/24" のようにCIDR付きで返るため先頭部分だけ使う
    return result.stdout.strip().split("/")[0]


def is_connected(iface: str = WLAN_IFACE) -> bool:
    """指定インターフェースがWi-Fi接続済み(NetworkManager state 100)かどうか。"""
    result = subprocess.run(
        ["nmcli", "-g", "GENERAL.STATE", "device", "show", iface],
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and result.stdout.strip().startswith("100")
