// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/StartMonitoring.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/StartMonitoring in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StartMonitoring_Request
{
  /// This service requests a monitor node to start monitoring a
  /// topic's incoming messages.
  /// The number of messages expected to be received during monitoring.
  /// The service returns early when all the expected messages are received.
  /// Ignored if set to 0.
  uint64_t message_count;
  /// Whether to use header.stamp.sec in each message's header as a message
  /// ID field
  bool revise_timestamps_as_message_ids;
  /// Whether to record timestamps in the incoming messages as start timestamps
  bool record_start_timestamps;
} ros2_benchmark_interfaces__srv__StartMonitoring_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StartMonitoring_Request.
typedef struct ros2_benchmark_interfaces__srv__StartMonitoring_Request__Sequence
{
  ros2_benchmark_interfaces__srv__StartMonitoring_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StartMonitoring_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/StartMonitoring in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StartMonitoring_Response
{
  uint8_t structure_needs_at_least_one_member;
} ros2_benchmark_interfaces__srv__StartMonitoring_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StartMonitoring_Response.
typedef struct ros2_benchmark_interfaces__srv__StartMonitoring_Response__Sequence
{
  ros2_benchmark_interfaces__srv__StartMonitoring_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StartMonitoring_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__STRUCT_H_
