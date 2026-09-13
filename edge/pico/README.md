# edge/pico/ 動作テスト一覧

`src/main.cpp` の `#include` を切り替えて、目的に応じたファームウェアをPicoへ書き込む。ピン配置は `doc/wiring.md` 参照。

`motor/` = モーター制御まわり、`imu/` = IMU(GY-521/MPU6050)まわりのテスト。`production/` は本番ファームウェアなのでテスト分類の外、`src/`直下に置く。

## 切り替え方

```cpp
// src/main.cpp
//#include "motor/one-motor/main.hpp"
//#include "motor/two-motor/main.hpp"
//#include "motor/ros-debug/main.hpp"
//#include "motor/alternate-motors/main.hpp"
//#include "imu/i2c-scan/main.hpp"
//#include "imu/mpu6050-lib-read/main.hpp"
//#include "imu/raw-read/main.hpp"
#include "production/main.hpp"   // ← 有効にしたいものだけコメントを外す
```

```sh
cd edge/pico
pio run -t upload
```

## 各ディレクトリの内容

| ディレクトリ | 内容 | ROS | モーター駆動 |
|---|---|---|---|
| `production` | 本番用。`/cmd_vel` を購読して差動二輪を駆動。agent切断時の自動再接続・安全停止に対応 | ○ | ○（両輪） |
| `motor/one-motor` | 片輪エンコーダのパルスカウントをSerialへ出力するだけの動作確認用 | × | × |
| `motor/two-motor` | 両輪エンコーダのパルスカウントをSerialへ出力するだけの動作確認用。モーターを一切駆動しないため、緊急停止用ファームとしても使える | × | × |
| `motor/ros-debug` | micro-ROSのagent接続・ノード初期化・サブスクライバー受信をLED点滅で可視化する診断用。モーターは動かさない | ○ | × |
| `motor/alternate-motors` | ROS抜きで左右モーターを交互に直接駆動。配線・TB6612自体の故障切り分け用 | × | ○（交互） |
| `imu/i2c-scan` | I2Cバスをスキャンし、応答するアドレスをSerialへ出力するだけの配線疎通確認用（GY-521/MPU6050向け） | × | × |
| `imu/mpu6050-lib-read` | GY-521(MPU6050)からAdafruit_MPU6050ライブラリ経由で加速度・角速度を読み、Serialへ出力する動作確認用 | × | × |
| `imu/raw-read` | GY-521(MPU6050)をライブラリ抜き・生のI2Cレジスタ読み書きだけで読む学習用。`mpu6050-lib-read`と同じ単位で出力し比較できる | × | × |
