# edge/pico/ 動作テスト一覧

`src/main.cpp` の `#include` を切り替えて、目的に応じたファームウェアをPicoへ書き込む。ピン配置は `doc/wiring.md` 参照。

## 切り替え方

```cpp
// src/main.cpp
//#include "one-motor/main.hpp"
//#include "two-motor/main.hpp"
//#include "ros-debug/main.hpp"
//#include "alternate-motors/main.hpp"
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
| `one-motor` | 片輪エンコーダのパルスカウントをSerialへ出力するだけの動作確認用 | × | × |
| `two-motor` | 両輪エンコーダのパルスカウントをSerialへ出力するだけの動作確認用。モーターを一切駆動しないため、緊急停止用ファームとしても使える | × | × |
| `ros-debug` | micro-ROSのagent接続・ノード初期化・サブスクライバー受信をLED点滅で可視化する診断用。モーターは動かさない | ○ | × |
| `alternate-motors` | ROS抜きで左右モーターを交互に直接駆動。配線・TB6612自体の故障切り分け用 | × | ○（交互） |
