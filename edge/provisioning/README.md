# edge/provisioning/ セットアップ手順

まっさらなラズパイをdiegobotが動く状態まで持っていくセットアップ自動化一式（Docker導入含む）と、
BLE経由のWi-Fiプロビジョニングデーモン（Dockerコンテナの外、ラズパイのホストOS上でsystemdサービスとして常駐）。

## 初回セットアップ

1. 開発機からラズパイへコードを転送

   ```sh
   make pi-rsync
   ```

2. ラズパイへSSHログイン

   ```sh
   make pi-ssh
   ```

3. ラズパイ上でセットアップスクリプトを実行

   ```sh
   cd ~/diegobot/edge/provisioning
   sudo ./setup.sh
   ```

   `setup.sh` が行うこと: Docker導入（未インストール時のみ、ログインユーザーをdockerグループへ追加） → APT依存関係インストール → venv作成・依存ライブラリインストール → 実行ユーザーをbluetoothグループへ追加 → systemdサービス登録・起動。

   Dockerグループの追加を反映させるには、実行後に一度SSHセッションを繋ぎ直す（再ログインする）必要がある。

### setup.sh と setup.dev.sh の使い分け

- `setup.sh`: 量産機にも必要な最小構成（Docker導入・BLEデーモン常駐化など）
- `setup.dev.sh`: `setup.sh`実行後に開発・検証用の追加ツール（`vim`など）も入れたい場合はこちらを実行する

   ```sh
   cd ~/diegobot/edge/provisioning
   sudo ./setup.dev.sh
   ```

## 動作確認

```sh
sudo systemctl status amr-provisioning.service
journalctl -u amr-provisioning.service -f
```

## コード更新時の再デプロイ

1. 開発機で `make pi-rsync`
2. ラズパイで `cd ~/diegobot/edge/provisioning && sudo ./setup.sh` を再実行

`setup.sh` は何度実行しても同じ結果になるように作ってあるので、更新のたびに1から手順を考える必要はなく、この2手順を繰り返すだけでよい。

## 設定変更

サービスの実行ユーザーを変えたい場合は `setup.sh` 冒頭の `SERVICE_USER`（デフォルト `root`）を書き換えてから再実行する。

## アンインストール

```sh
sudo systemctl disable --now amr-provisioning.service
sudo rm /etc/systemd/system/amr-provisioning.service
sudo systemctl daemon-reload
```
