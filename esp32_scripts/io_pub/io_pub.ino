#include "LIS2MDL.h"
#include "LSM6DSL.h"
#include "DFRobot_GNSS.h"

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
#include <sensor_msgs/msg/nav_sat_fix.h>

#include <rcutils/logging_macros.h>

// Create sensor objects
LSM6DSL imu;
LIS2MDL mag;
DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);

// ROS 2 components
typedef struct {
  rclc_support_t support;
  rcl_node_t node;
  rcl_publisher_t imu_pub;
  rcl_publisher_t mag_pub;
  rcl_publisher_t gnss_pub;
  rcl_timer_t timer;
  rclc_executor_t executor;
  rcl_allocator_t allocator;
} ros2_components;

ros2_components ros2;

// Message instances
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
sensor_msgs__msg__NavSatFix gnss_msg;

// Timer callback: read sensors, stamp, publish
void callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);

}

void setup() {
  // Initialize GNSS Module
  gnss.enablePower(); 
  gnss.setGnss(eGPS_BeiDou_GLONASS);
  gnss.setRgbOn();

  set_microros_transports();

  // 1) Initialize support & allocator
  ros2.allocator = rcl_get_default_allocator();
  rclc_support_init(&ros2.support, 0, NULL, &ros2.allocator);

  // 2) Sync time with agent (max 1s)
  if (RMW_RET_OK != rmw_uros_sync_session(1000)) {
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
  rclc_publisher_init_default(
    &ros2.gnss_pub,
    &ros2.node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
    "gnss"
  );

  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__MagneticField__init(&mag_msg);
  sensor_msgs__msg__NavSatFix__init(&gnss_msg);

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
  const unsigned long period_ms = 20;
  rclc_timer_init_default(
    &ros2.timer,
    &ros2.support,
    RCL_MS_TO_NS(period_ms),
    callback
  );

  // 7) Executor
  rclc_executor_init(&ros2.executor, &ros2.support.context, 2, &ros2.allocator);
  rclc_executor_add_timer(&ros2.executor, &ros2.timer);

  const char * frame = "imu_link";
  imu_msg.header.frame_id.data     = (char *)frame;
  imu_msg.header.frame_id.size     = strlen(frame);
  imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;

  const char * gnss_frame = "gnss_link";
  gnss_msg.header.frame_id.data = (char *)gnss_frame;
  gnss_msg.header.frame_id.size = strlen(gnss_frame);
  gnss_msg.header.frame_id.capacity = gnss_msg.header.frame_id.size;
}

void loop() {
  rmw_uros_sync_session(20);
  int64_t now_ns = rmw_uros_epoch_nanos();
  // Spin executor
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  sTim_t utc = gnss.getUTC();
  sTim_t date = gnss.getDate();
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
  double high = gnss.getAlt();
  uint8_t starUserd = gnss.getNumSatUsed();
  double sog = gnss.getSog();
  double cog = gnss.getCog();

  imu.readData(ax, ay, az, gx, gy, gz);
  mag.readData(mx, my, mz);

  // 1) Timestamp via synchronized clock
  

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

  // 5) Prepare GNSS message

  gnss_msg.latitude = lat.latitudeDegree;
  gnss_msg.longitude = lon.lonitudeDegree;
  gnss_msg.altitude = high;

  gnss_msg.position_covariance_type =
    sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;
  

  // 6) Publish both topics
  
  // rclc_executor_spin_some(&ros2.executor, RCL_MS_TO_NS(1));
  imu_msg.header.stamp.sec     = now_ns / 1000000000LL;
  imu_msg.header.stamp.nanosec = now_ns % 1000000000LL;
  gnss_msg.header.stamp.sec  = utc.hour * 3600 + utc.minute * 60 + utc.second;

  mag_msg.header = imu_msg.header;

  rcl_publish(&ros2.imu_pub, &imu_msg, NULL);
  rcl_publish(&ros2.mag_pub, &mag_msg, NULL);
  rcl_publish(&ros2.gnss_pub, &gnss_msg, NULL);
}
