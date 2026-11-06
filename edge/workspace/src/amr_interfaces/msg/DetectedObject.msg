string label
float32 score

# カラー画像上の外接矩形[px]
vision_msgs/BoundingBox2D bbox

# 位置が求まらなかった場合はfalseで、position・width・heightは無効。マスクに深度が全く無いときは床に置かれた物とみなして
# positionだけ求める(width・heightは0のまま)。
bool position_valid
# マスク内の深度の中央値と、深度のあるマスク画素の重心から求めた点。物体の中心ではなくカメラに向いた表面上の点。
geometry_msgs/PointStamped position
# 外接矩形を深度の距離で換算した、カメラから見た幅・高さ[m]
float32 width
float32 height

# マスク内で最も多い色の名前(red, orange, yellow, green, cyan, blue, purple, pink, white, gray, black)
string color
