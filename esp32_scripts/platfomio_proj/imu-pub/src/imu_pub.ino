  #include "LIS2MDL.h"
  #include "LSM6DSL.h"

  #include <Arduino.h>
  #include <Wire.h>
  #include <string.h>

  #include <micro_ros_platformio.h>
  #include <rmw_microros/rmw_microros.h>
  #include <rcl/rcl.h>
  #include <rcl/error_handling.h>
  #include <rcl/time.h>
  #include <rclc/rclc.h>
  #include <rclc/executor.h>

  #include <sensor_msgs/msg/imu.h>
  #include <sensor_msgs/msg/magnetic_field.h>

  #include <rcutils/logging_macros.h>

  #include <micro_ros_utilities/string_utilities.h>

  #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  #error This example is only avaliable for Arduino framework with serial transport.
  #endif


  #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
  #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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

  int64_t now_ns;

  // Timer callback: read sensors, stamp, publish
  void callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED (timer);
    

    // Spin executor
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    imu.readData(ax, ay, az, gx, gy, gz, true);
    mag.readData(mx, my, mz, true);
    // 1) Timestamp via synchronized clock

    now_ns = rmw_uros_epoch_nanos();

    if (now_ns == 0ULL) {
    // try a quick re-sync without blocking the timer too long
      rmw_uros_sync_session(5);
      now_ns = rmw_uros_epoch_nanos();
    }

    imu_msg.header.stamp.sec     = now_ns / 1000000000LL;
    imu_msg.header.stamp.nanosec = now_ns % 1000000000LL;
    // imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
    mag_msg.header = imu_msg.header;

    // 3) Fill IMU data (casts to double)
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x    = gx;
    imu_msg.angular_velocity.y    = gy;
    imu_msg.angular_velocity.z    = gz;
    // 4) Prepare magnetometer message
    
    mag_msg.magnetic_field.x = mx;
    mag_msg.magnetic_field.y = my;
    mag_msg.magnetic_field.z = mz; 

    RCSOFTCHECK(rcl_publish(&ros2.imu_pub, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&ros2.mag_pub, &mag_msg, NULL));
  }

  void setup() {
    Serial.begin(921600);
    set_microros_serial_transports(Serial);
    delay(2000);

    // 1) Initialize support & allocator
    ros2.allocator = rcl_get_default_allocator();
    rclc_support_init(&ros2.support, 0, NULL, &ros2.allocator);

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
    Wire.begin(8, 10);
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

    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

    rmw_uros_sync_session(1000);
  }

  void loop() {
    
    
    rclc_executor_spin_some(&ros2.executor, RCL_MS_TO_NS(10));
  }