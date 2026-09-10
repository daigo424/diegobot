#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>

// production/main.hppからサブスクライバー・モーター制御を除いた最小構成。
// agent側のセッション確立(create_client/create_participant)だけを検証する。

const int LED_PIN = 25;

// production/main.hppと同じモーターピン初期化 (犯人候補として先に実行して再現するか検証)
const int PIN_PWMA = 4;
const int PIN_AIN1 = 2;
const int PIN_AIN2 = 3;
const int PIN_PWMB = 8;
const int PIN_BIN1 = 6;
const int PIN_BIN2 = 7;
const int PWM_FREQ  = 20000;
const int PWM_RANGE = 255;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;

void subscription_callback(const void * msgin) {
  // 受信の可視化のみ (モーターは駆動しない)
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

void _setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // agent到達性をタイムアウト付きで確認 (ここでハングしない)
  rmw_ret_t ping_result = rmw_uros_ping_agent(1000, 3);
  if (ping_result != RMW_RET_OK) {
    // 到達不可: LED高速点滅を無限ループで示す
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  // ping成功: LED長押し点灯で知らせる
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "rppico_debug_node", "", &support);

  // ノード初期化まで完了: LED 3回点滅
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  // サブスクライバー・エグゼキューター初期化まで完了: LED 5回点滅
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void _loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}

#endif // MAIN_HPP
