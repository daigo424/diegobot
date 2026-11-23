// 上下逆さに付けたカメラの画像とcamera_infoを180°回して出し直す。
// 回した画像のframe_idは元のframe_idに"_upright"を付けたもので、元のフレームを光軸回りに180°回した向きとしてTFに出す。
// 深度画像も同じく回すため、camera_infoとframe_idから3次元に戻す側は回したことを意識しなくてよい。
#include <algorithm>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace amr_bringup
{

class RotateImage : public rclcpp::Node
{
public:
  explicit RotateImage(const rclcpp::NodeOptions & options)
  : Node("rotate_image", options), tf_(this)
  {
    auto streams = declare_parameter<std::vector<std::string>>("streams", {"color", "depth"});
    for (const auto & stream : streams) {
      auto image_pub = create_publisher<sensor_msgs::msg::Image>(stream + "/image", 5);
      auto info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(stream + "/camera_info", 5);
      image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
          stream + "/image_in", rclcpp::SensorDataQoS(),
          [this, image_pub](sensor_msgs::msg::Image::UniquePtr msg) {
            // Pi 4ではカメラの処理で手一杯になりやすいため、誰も見ていない画像は回さない。
            if (image_pub->get_subscription_count() > 0) {
              image_pub->publish(rotate(std::move(msg)));
            }
          }));
      info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
          stream + "/camera_info_in", rclcpp::SensorDataQoS(),
          [this, info_pub](sensor_msgs::msg::CameraInfo::UniquePtr msg) {
            if (info_pub->get_subscription_count() > 0) {
              info_pub->publish(rotate(std::move(msg)));
            }
          }));
    }
  }

private:
  std::string upright_frame(const std::string & frame)
  {
    if (static_.count(frame) == 0) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = frame;
      t.child_frame_id = frame + "_upright";
      t.transform.rotation.z = 1.0;
      t.transform.rotation.w = 0.0;
      static_[frame] = t;
      // StaticTransformBroadcasterは送るたびに前の分を置き換えるため、これまでの分もまとめて送る。
      std::vector<geometry_msgs::msg::TransformStamped> all;
      for (const auto & kv : static_) {
        all.push_back(kv.second);
      }
      tf_.sendTransform(all);
    }
    return frame + "_upright";
  }

  sensor_msgs::msg::Image::UniquePtr rotate(sensor_msgs::msg::Image::UniquePtr msg)
  {
    const size_t bpp = msg->width ? msg->step / msg->width : 0;
    const size_t row = msg->width * bpp;
    std::vector<uint8_t> out(row * msg->height);
    for (size_t y = 0; y < msg->height; ++y) {
      const uint8_t * src = msg->data.data() + y * msg->step;
      uint8_t * dst = out.data() + (msg->height - 1 - y) * row;
      for (size_t x = 0; x < msg->width; ++x) {
        std::memcpy(dst + (msg->width - 1 - x) * bpp, src + x * bpp, bpp);
      }
    }
    msg->data = std::move(out);
    msg->step = row;
    msg->header.frame_id = upright_frame(msg->header.frame_id);
    return msg;
  }

  sensor_msgs::msg::CameraInfo::UniquePtr rotate(sensor_msgs::msg::CameraInfo::UniquePtr msg)
  {
    const double w = msg->width, h = msg->height;
    msg->k[2] = w - 1 - msg->k[2];
    msg->k[5] = h - 1 - msg->k[5];
    msg->p[2] = w - 1 - msg->p[2];
    msg->p[6] = h - 1 - msg->p[6];
    // 像の上下左右が入れ替わると、接線方向の歪み(p1, p2)だけ符号が変わる。
    if ((msg->distortion_model == "plumb_bob" || msg->distortion_model == "rational_polynomial") &&
      msg->d.size() >= 4)
    {
      msg->d[2] = -msg->d[2];
      msg->d[3] = -msg->d[3];
    }
    msg->header.frame_id = upright_frame(msg->header.frame_id);
    return msg;
  }

  tf2_ros::StaticTransformBroadcaster tf_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> static_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> info_subs_;
};

}  // namespace amr_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(amr_bringup::RotateImage)
