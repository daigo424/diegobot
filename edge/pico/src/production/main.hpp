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
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>

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

// --- ピン定義・車輪オドメトリ (クアドラチャエンコーダ) ---
const int PIN_LEFT_ENC_A  = 18;
const int PIN_LEFT_ENC_B  = 19;
const int PIN_RIGHT_ENC_A = 16;
const int PIN_RIGHT_ENC_B = 17;

// 車輪軸換算CPR(モーター軸11パルス/回転を4逓倍・56:1減速で換算)・車輪直径(68mm、実測)からの距離換算。
const float COUNTS_PER_REV = 2464.0f;
const float WHEEL_DIAMETER_M = 0.068f;
const float METERS_PER_COUNT = (WHEEL_DIAMETER_M * PI) / COUNTS_PER_REV;
const float RADIANS_PER_COUNT = (2.0f * PI) / COUNTS_PER_REV;

// 4逓倍クアドラチャデコード用の状態遷移テーブル。
// インデックスは(前回のAB 2bit << 2 | 今回のAB 2bit)の4bit。
// 値は正しい遷移なら+1/-1、あり得ない遷移(パルス取りこぼし時に起き得る)なら0。
const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0,
};

volatile long left_encoder_count = 0;
volatile long right_encoder_count = 0;
volatile uint8_t left_encoder_state = 0;
volatile uint8_t right_encoder_state = 0;

void update_left_encoder() {
  left_encoder_state = ((left_encoder_state << 2) | (digitalRead(PIN_LEFT_ENC_A) << 1) | digitalRead(PIN_LEFT_ENC_B)) & 0x0F;
  left_encoder_count += QEM[left_encoder_state];
}

void update_right_encoder() {
  right_encoder_state = ((right_encoder_state << 2) | (digitalRead(PIN_RIGHT_ENC_A) << 1) | digitalRead(PIN_RIGHT_ENC_B)) & 0x0F;
  right_encoder_count += QEM[right_encoder_state];
}

// オドメトリの積算姿勢(ロボット座標系)。原点はagent接続確立時点。
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;
long prev_left_count = 0;
long prev_right_count = 0;
int64_t prev_odom_ms = 0;

// micro-ROS 関連オブジェクト
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_state_msg;
// joint_state_msg.name/position/velocityが指すバッキングストア。動的確保(Sequence__init)を
// 避け、_setup()で一度だけ配線してrcl_publish()ごとの値の書き換えだけで済むようにする。
rosidl_runtime_c__String joint_state_names[2];
double joint_state_positions[2];
double joint_state_velocities[2];
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

// create_entities()でのrmw_uros_sync_session()によるagentとのエポック時刻同期後の値を
// header.stampに詰める。
void stamp_header(std_msgs__msg__Header &header) {
  int64_t epoch_ns = rmw_uros_epoch_nanos();
  header.stamp.sec = epoch_ns / 1000000000LL;
  header.stamp.nanosec = epoch_ns % 1000000000LL;
}

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

  if (rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states") != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // header.stampをagentのエポック時刻に合わせる。失敗しても以降0付近の値になるだけで
  // publish自体は継続するため、戻り値は見ない。
  (void) rmw_uros_sync_session(1000);

  // 再接続時に切断中の移動分をまとめて1回の速度として計算してしまわないよう、
  // 接続確立のタイミングでオドメトリの基準点をリセットする。
  noInterrupts();
  prev_left_count = left_encoder_count;
  prev_right_count = right_encoder_count;
  interrupts();
  prev_odom_ms = uxr_millis();

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&odom_publisher, &node);
  rcl_publisher_fini(&joint_state_publisher, &node);
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

  stamp_header(imu_msg.header);
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
  // 未キャリブレーションの暫定値(tasks/robot-assembly.mdでキャリブレーション後に実測値へ更新)。
  // 全て0のままだと「無限に信頼できる測定値」とEKFに解釈されてしまうため、必ず非ゼロにする。
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  rcl_publish(&imu_publisher, &imu_msg, NULL);
}

// エンコーダ実測値から差動二輪の推測航法(dead reckoning)でオドメトリを計算し、
// /odomへpublishする。TFのbroadcastはここでは行わない(将来EKFノード側の責務にする)。
void publish_odom() {
  // 割り込みで更新中の値を短時間だけ止めて一貫性のあるスナップショットを取る。
  noInterrupts();
  long left_count = left_encoder_count;
  long right_count = right_encoder_count;
  interrupts();

  int64_t now_ms = uxr_millis();
  float dt = (now_ms - prev_odom_ms) / 1000.0f;
  prev_odom_ms = now_ms;
  if (dt <= 0.0f) return;

  long left_delta = left_count - prev_left_count;
  long right_delta = -(right_count - prev_right_count); // 右モーターは鏡像取り付けのため符号反転(-right_ratioと同じ考え方)
  prev_left_count = left_count;
  prev_right_count = right_count;

  float left_dist  = left_delta * METERS_PER_COUNT;
  float right_dist = right_delta * METERS_PER_COUNT;

  float delta_center = (left_dist + right_dist) / 2.0f;
  float delta_theta  = (right_dist - left_dist) / WHEEL_BASE;

  // 区間中点でのthetaを使って積分することで、大きめの区間でも直線移動をわずかな
  // 円弧として近似でき、単純なオイラー積分より誤差が小さくなる。
  float theta_mid = odom_theta + delta_theta / 2.0f;
  odom_x += delta_center * cos(theta_mid);
  odom_y += delta_center * sin(theta_mid);
  odom_theta += delta_theta;

  stamp_header(odom_msg.header);
  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = 4;
  odom_msg.header.frame_id.capacity = 5;
  odom_msg.child_frame_id.data = (char *)"base_footprint";
  odom_msg.child_frame_id.size = 14;
  odom_msg.child_frame_id.capacity = 15;

  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0f);
  odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0f);
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;

  odom_msg.twist.twist.linear.x = delta_center / dt;
  odom_msg.twist.twist.angular.z = delta_theta / dt;

  // 共分散は暫定値。差動二輪では観測できない次元(z, roll, pitch, linear.y等)は
  // 大きな値にしてEKF側に「信用しない」ことを伝える。x/y/yaw/vx/wzは走行テストで
  // 実測して調整すること。
  odom_msg.pose.covariance[0]  = 0.01;  // x
  odom_msg.pose.covariance[7]  = 0.01;  // y
  odom_msg.pose.covariance[14] = 1e6;   // z
  odom_msg.pose.covariance[21] = 1e6;   // roll
  odom_msg.pose.covariance[28] = 1e6;   // pitch
  odom_msg.pose.covariance[35] = 0.01;  // yaw

  odom_msg.twist.covariance[0]  = 0.01; // vx
  odom_msg.twist.covariance[7]  = 1e6;  // vy
  odom_msg.twist.covariance[14] = 1e6;  // vz
  odom_msg.twist.covariance[21] = 1e6;  // wx
  odom_msg.twist.covariance[28] = 1e6;  // wy
  odom_msg.twist.covariance[35] = 0.01; // wz

  rcl_publish(&odom_publisher, &odom_msg, NULL);

  // wheel_left_link/wheel_right_linkはURDF上type="continuous"のため、robot_state_publisherは
  // /joint_statesが無いとTFをbroadcastしない(fixed jointと違い角度をここから貰う必要がある)。
  stamp_header(joint_state_msg.header);
  joint_state_msg.position.data[0] = left_count * RADIANS_PER_COUNT;
  joint_state_msg.position.data[1] = -right_count * RADIANS_PER_COUNT; // 右輪の符号反転はodom_msgと同じ理由
  joint_state_msg.velocity.data[0] = (left_delta * RADIANS_PER_COUNT) / dt;
  joint_state_msg.velocity.data[1] = (right_delta * RADIANS_PER_COUNT) / dt;
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
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

  // エンコーダ初期化
  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), update_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_B), update_left_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), update_right_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_B), update_right_encoder, CHANGE);

  // joint_state_msgのname/position/velocityを静的ストレージに配線(値そのものは
  // publish_odom()内で毎回更新するので、ここでは一度だけでよい)。
  joint_state_names[0].data = (char *)"wheel_left_joint";
  joint_state_names[0].size = 16;
  joint_state_names[0].capacity = 17;
  joint_state_names[1].data = (char *)"wheel_right_joint";
  joint_state_names[1].size = 17;
  joint_state_names[1].capacity = 18;
  joint_state_msg.name.data = joint_state_names;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.capacity = 2;
  joint_state_msg.position.data = joint_state_positions;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.capacity = 2;
  joint_state_msg.velocity.data = joint_state_velocities;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.capacity = 2;

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
        EXECUTE_EVERY_N_MS(20, publish_odom();); // 50Hz
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
