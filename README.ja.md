# Diegobot

[English version here](README.md)

自作の差動二輪AMR（自律移動ロボット）。ROS 2 Jazzy + micro-ROSで足回りを制御し、スマホから遠隔操縦できる。

最終的にはTurtleBot3 Waffleのような自律移動ロボットの自作を目指す。

ハードウェア詳細は [doc/wiring.md](doc/wiring.md) を参照。

## 構成

```text
diegobot/
├── edge/
│   ├── docker/         # ROS 2 Jazzyイメージ・rosbridge・micro-ROS Agentのcompose定義
│   ├── workspace/      # ROS 2ワークスペース(colcon)
│   ├── pico/           # Picoファームウェア(PlatformIO)。本番/動作確認テスト一式 → edge/pico/README.md
│   ├── esp32/          # 旧ESP32版ファームウェア(レガシー。現在はPicoへ移行済み)
│   └── provisioning/   # ラズパイの初期セットアップ自動化 + BLE Wi-Fiプロビジョニングデーモン → edge/provisioning/README.md
├── mobile/              # Flutterアプリ「diegoctl」(BLEプロビジョニング + ジョイスティック操縦)
├── doc/                 # 配線などハードウェアドキュメント
└── Makefile             # 開発・デプロイの各種操作をまとめたエントリポイント
```

## 開発機での基本操作

```sh
make build      # ROS 2イメージをビルド
make up         # コンテナ群を起動 (diegobot / rosbridge / micro-ros-agent)
make down       # 停止
make login      # diegobotコンテナへ入る
```

モーター動作確認:

```sh
make teleop-twist-keyboard   # キーボードで/cmd_velを送って走行確認
```

`edge/docker/docker-compose.yml`の`micro-ros-agent`サービスは、ホストのUSBシリアルポート(Pico)を`.env`の`HOST_USB_PORT`で指定する。`/dev/ttyACM*`の番号は抜き差しやreflashのたびにずれることがあるため、都度確認すること。

## Picoファームウェア

`edge/pico/` にPlatformIOプロジェクトがあり、本番用ファームウェアに加えてモーター単体テスト等の動作確認用ファームウェアも一式入っている。切り替え方・各テストの内容は [edge/pico/README.md](edge/pico/README.md) を参照。

```sh
cd edge/pico
pio run -t upload
```

## ラズパイへのデプロイ

```sh
make pi-rsync   # コード転送
make pi-ssh     # ラズパイへSSH
```

ラズパイ上での初回セットアップ(Docker導入・BLEプロビジョニングデーモン常駐化など)は [edge/provisioning/README.md](edge/provisioning/README.md) を参照。

コンテナイメージを開発機側でクロスビルドしてラズパイへ転送したい場合(ラズパイ上でのビルドが遅い/メモリ不足になりがちな場合向け):

```sh
make pi-push-image
```

## モバイルアプリ (diegoctl)

`mobile/` にFlutterアプリがある。BLE経由のWi-Fiプロビジョニング画面と、rosbridge経由で`/cmd_vel`を送るジョイスティック操縦画面を持つ。

```sh
make android-install   # リリースAPKをビルドして接続中の実機へインストール
```
