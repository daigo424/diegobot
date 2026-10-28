# edge/provisioning/ セットアップ手順

まっさらなラズパイをdiegobotが動く状態まで持っていくセットアップ自動化一式（Docker導入含む）と、
BLE経由のWi-Fiプロビジョニングデーモン（Dockerコンテナの外、ラズパイのホストOS上でsystemdサービスとして常駐）。

## 初回セットアップ

1. 開発機からラズパイへコードを転送

   ```sh
   make pi-rsync
   ```

2. 開発機からラズパイ上のセットアップスクリプトを実行（ラズパイ上で直接 `cd ~/diegobot/edge/provisioning && sudo ./setup.sh` しても同じ）

   ```sh
   make pi-setup
   ```

   `setup.sh` が行うこと: Docker導入（未インストール時のみ、ログインユーザーをdockerグループへ追加） → APT依存関係インストール → venv作成・依存ライブラリインストール → 実行ユーザーをbluetoothグループへ追加 → systemdサービス登録・起動 → USBデバイス(深度カメラ等)のudevルール導入 → 起動時のコンテナ自動起動サービス(`diegobot.service`)登録。

   Dockerグループの追加を反映させるには、実行後に一度SSHセッションを繋ぎ直す（再ログインする）必要がある。

### setup.sh と setup.dev.sh の使い分け

- `setup.sh`: 量産機にも必要な最小構成（Docker導入・BLEデーモン常駐化など）
- `setup.dev.sh`: `setup.sh`実行後に開発・検証用の追加ツール（`vim`など）も入れたい場合はこちらを実行する

   ```sh
   make pi-setup-dev
   ```

## 起動時のコンテナ自動起動

`diegobot.service` がラズパイ起動時に `make up-boot` を実行し、コンテナ群(diegobot / rosbridge / micro-ros-agent)を立ち上げる。`make up` と違いパッケージインストールは行わない。`.env` の `HOST_MICRO_ROS_PORT` / `HOST_LIDAR_PORT` / `HOST_ARM_PORT` が未接続ならそのデバイス無しで起動する（`make up` も同様）。`setup.sh` 実行直後ではなく、次回の起動から有効になる。

## 動作確認

```sh
sudo systemctl status amr-provisioning.service
journalctl -u amr-provisioning.service -f
sudo systemctl status diegobot.service
journalctl -u diegobot.service
```

## コード更新時の再デプロイ

1. 開発機で `make pi-rsync`
2. 開発機で `make pi-setup` を再実行

`setup.sh` は何度実行しても同じ結果になるように作ってあるので、更新のたびに1から手順を考える必要はなく、この2手順を繰り返すだけでよい。

## 設定変更

サービスの実行ユーザーを変えたい場合は `setup.sh` 冒頭の `SERVICE_USER`（デフォルト `root`）を書き換えてから再実行する。

## アンインストール

```sh
sudo systemctl disable --now amr-provisioning.service diegobot.service
sudo rm /etc/systemd/system/amr-provisioning.service /etc/systemd/system/diegobot.service
sudo systemctl daemon-reload
```
