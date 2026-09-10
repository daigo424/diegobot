#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// --- ピン定義 (TB6612FNG) ---
// 左モーター (Motor A)
const int PIN_PWMA = 25;
const int PIN_AIN1 = 26;
const int PIN_AIN2 = 27;

// 右モーター (Motor B)
const int PIN_PWMB = 14;
const int PIN_BIN1 = 12;
const int PIN_BIN2 = 13;

// PWMチャンネル設定 (ESP32 LEDC)
const int PWM_CH_LEFT  = 0;
const int PWM_CH_RIGHT = 1;
const int PWM_FREQ     = 20000; // 20kHz (可聴域外で静音化)
const int PWM_RES      = 8;     // 8bit (0〜255)

// 車体パラメータ
const float WHEEL_BASE = 0.17;  // トレッド幅（左右輪の間隔: 約18cm）
const float MAX_SPEED  = 0.5;   // 最大想定速度 (m/s)

// micro-ROS 関連オブジェクト
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// モーター個別制御関数
void set_motor(int ch, int pin_in1, int pin_in2, float speed_ratio) {
  // speed_ratio: -1.0 (最大逆転) 〜 1.0 (最大正転)
  speed_ratio = constrain(speed_ratio, -1.0f, 1.0f);
  int duty = abs(speed_ratio) * 255;

  if (speed_ratio > 0.05f) {
    // 正転
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
    ledcWrite(ch, duty);
  } else if (speed_ratio < -0.05f) {
    // 逆転
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
    ledcWrite(ch, duty);
  } else {
    // 停止（ショートブレーキ）
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
    ledcWrite(ch, 0);
  }
}

// /cmd_vel 受信コールバック
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear_x  = twist_msg->linear.x;   // 前進/後退速度 (m/s)
  float angular_z = twist_msg->angular.z;  // 旋回角速度 (rad/s)

  // 差動二輪の運動学計算 (左右車輪の目標並進速度)
  float left_vel  = linear_x - (angular_z * WHEEL_BASE / 2.0f);
  float right_vel = linear_x + (angular_z * WHEEL_BASE / 2.0f);

  // 速度から -1.0〜1.0 の比率へ正規化
  float left_ratio  = left_vel / MAX_SPEED;
  float right_ratio = right_vel / MAX_SPEED;

  set_motor(PWM_CH_LEFT, PIN_AIN1, PIN_AIN2, left_ratio);
  set_motor(PWM_CH_RIGHT, PIN_BIN1, PIN_BIN2, -right_ratio);
}

void _setup() {
  // モーターピン初期化
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // PWMチャンネル初期化
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMA, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWMB, PWM_CH_RIGHT);

  // micro-ROS トランスポート初期化
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // micro-ROS ノードとサブスクライバーの作成
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_base_controller", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void _loop() {
  // micro-ROS イベントを定期実行
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}
