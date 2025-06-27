#include "LIS2MDL.h"
#include "LSM6DSL.h"

#include <Wire.h>
#include <string.h>

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/time.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <rcutils/logging_macros.h>

// Create sensor objects
LSM6DSL imu;
LIS2MDL mag;

// ROS 2 components
typedef struct {
  rclc_support_t support;
  rcl_node_t node;
  rcl_publisher_t imu_pub;
  rcl_publisher_t mag_pub;
  rcl_timer_t timer;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
} ros2_components;

ros2_components ros2;

// Message instances
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;

// Timer callback: read sensors, stamp, publish
void callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;

  imu.readData(ax, ay, az, gx, gy, gz);
  mag.readData(mx, my, mz);

  // 1) Timestamp via synchronized clock
  int64_t now_ns = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec     = now_ns / 1000000000LL;
  imu_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

  // 2) Frame ID
  const char * frame = "imu_link";
  imu_msg.header.frame_id.data     = (char *)frame;
  imu_msg.header.frame_id.size     = strlen(frame);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  // 3) Fill IMU data (casts to double)
  imu_msg.linear_acceleration.x = (double)ax;
  imu_msg.linear_acceleration.y = (double)ay;
  imu_msg.linear_acceleration.z = (double)az;
  imu_msg.angular_velocity.x    = (double)gx;
  imu_msg.angular_velocity.y    = (double)gy;
  imu_msg.angular_velocity.z    = (double)gz;

  // 4) Prepare magnetometer message
  mag_msg.header = imu_msg.header;
  mag_msg.magnetic_field.x = (double)mx;
  mag_msg.magnetic_field.y = (double)my;
  mag_msg.magnetic_field.z = (double)mz;

  // 5) Debug log
  RCUTILS_LOG_INFO_NAMED(
    "esp32_node",
    "Accel: %.2f, %.2f, %.2f | Gyro: %.2f, %.2f, %.2f | Mag: %.2f, %.2f, %.2f",
    ax, ay, az, gx, gy, gz, mx, my, mz
  );

  // 6) Publish both topics
  rcl_publish(&ros2.imu_pub, &imu_msg, NULL);
  rcl_publish(&ros2.mag_pub, &mag_msg, NULL);
}

void setup() {
  // Serial for transport & debug
  Serial.begin(115200);
  while (!Serial) { delay(10); }
    // Configure micro-ROS to use Serial transport via UART0
  rmw_uros_set_custom_transport(
    true,
    &Serial,
    arduino_transport_open,
    arduino_transport_close,
    arduino_transport_write,
    arduino_transport_read
  );

  // 1) Initialize support & allocator
  ros2.allocator = rcl_get_default_allocator();
  rclc_support_init(&ros2.support, 0, NULL, &ros2.allocator);

  // 2) Sync time with agent (max 1s)
  if (RMW_RET_OK != rmw_uros_sync_session(200)) {
    RCUTILS_LOG_ERROR_NAMED("esp32_node", "Time sync failed");
  }

  // 3) Create node
  rclc_node_init_default(&ros2.node, "esp32_node", "", &ros2.support);

  // 4) Create publishers
  rclc_publisher_init_default(
    &ros2.imu_pub,
    &ros2.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"
  );
  rclc_publisher_init_default(
    &ros2.mag_pub,
    &ros2.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "mag"
  );

  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__MagneticField__init(&mag_msg);

  // 5) Initialize I2C & sensors
  Wire.begin();
  if (!imu.beginI2C(Wire)) {
    RCUTILS_LOG_ERROR_NAMED("esp32_node", "LSM6DSL init failed");
    while (1) { delay(1000); }
  }
  if (!mag.beginI2C(Wire)) {
    RCUTILS_LOG_ERROR_NAMED("esp32_node", "LIS2MDL init failed");
    while (1) { delay(1000); }
  }

  // 6) Create timer (50 Hz)
  const unsigned long period_ms = 10;
  rclc_timer_init_default(
    &ros2.timer,
    &ros2.support,
    RCL_MS_TO_NS(period_ms),
    callback
  );

  // 7) Executor
  rclc_executor_init(&ros2.executor, &ros2.support.context, 1, &ros2.allocator);
  rclc_executor_add_timer(&ros2.executor, &ros2.timer);
}

void loop() {
  // Spin executor
  rclc_executor_spin(&ros2.executor);
}
