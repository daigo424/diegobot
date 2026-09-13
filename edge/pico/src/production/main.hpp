#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

// --- ピン定義 (TB6612FNG) ---
// GP23/24/25/29はPicoボード上でSMPSモード制御・VBUS検知・LED・VSYS電圧センスに
// 専用配線されているため使わない。
// 左モーター (Motor A)
const int PIN_PWMA = 4;
const int PIN_AIN1 = 2;
const int PIN_AIN2 = 3;

// 右モーター (Motor B)
const int PIN_PWMB = 8;
const int PIN_BIN1 = 6;
const int PIN_BIN2 = 7;

// PWM設定 (RP2040は全PWM出力ピンでfreq/rangeを共有する)
const int PWM_FREQ  = 20000; // 20kHz (可聴域外で静音化)
const int PWM_RANGE = 255;   // 8bit相当 (0〜255)

// 車体パラメータ
const float WHEEL_BASE = 0.17;  // トレッド幅（左右輪の間隔: 約18cm）
const float MAX_SPEED  = 0.5;   // 最大想定速度 (m/s)

// --- ピン定義 (GY-521 / MPU6050) ---
const int PIN_SDA = 20;
const int PIN_SCL = 21;
Adafruit_MPU6050 mpu;
bool imu_available = false;

// micro-ROS 関連オブジェクト
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// RP2040のUSB CDCシリアルトランスポートはagentへのping送出なしに放置すると
// 接続確立から十数秒でハングし復帰しない (USB UART変換チップ経由のESP32では発生しなかった)。
// 公式のmicro-ROS再接続パターンでagentの生存を能動的に監視する。
enum State {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
State state;

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

void stop_motors() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

// モーター個別制御関数
void set_motor(int pin_pwm, int pin_in1, int pin_in2, float speed_ratio) {
  // speed_ratio: -1.0 (最大逆転) 〜 1.0 (最大正転)
  speed_ratio = constrain(speed_ratio, -1.0f, 1.0f);
  int duty = abs(speed_ratio) * PWM_RANGE;

  if (speed_ratio > 0.05f) {
    // 正転
    digitalWrite(pin_in1, HIGH);
    digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, duty);
  } else if (speed_ratio < -0.05f) {
    // 逆転
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, HIGH);
    analogWrite(pin_pwm, duty);
  } else {
    // 停止（ショートブレーキ）
    digitalWrite(pin_in1, LOW);
    digitalWrite(pin_in2, LOW);
    analogWrite(pin_pwm, 0);
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

  set_motor(PIN_PWMA, PIN_AIN1, PIN_AIN2, left_ratio);
  set_motor(PIN_PWMB, PIN_BIN1, PIN_BIN2, -right_ratio);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "rppico_base_controller", "", &support) != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu") != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// IMUを読んで/imuへpublishする。orientationは未推定(センサーフュージョン無し)のため
// REP-145の規約通りorientation_covariance[0]=-1で「値なし」を明示する。
void publish_imu() {
  if (!imu_available) return;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  imu_msg.header.frame_id.data = (char *)"imu_link";
  imu_msg.header.frame_id.size = 8;
  imu_msg.header.frame_id.capacity = 9;

  imu_msg.orientation.w = 1.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation_covariance[0] = -1;

  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;

  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

void _setup() {
  // モーターピン初期化
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // PWM初期化 (RP2040はグローバル設定、ピンごとの周波数指定はできない)
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  stop_motors();

  // IMU初期化。見つからなくてもモーター制御自体は継続する。
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();
  imu_available = mpu.begin();

  // micro-ROS トランスポート初期化
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  state = WAITING_AGENT;
}

void _loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        EXECUTE_EVERY_N_MS(50, publish_imu();); // 20Hz
      }
      break;

    case AGENT_DISCONNECTED:
      // agentを見失った状態でモーターを回し続けない
      stop_motors();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}
