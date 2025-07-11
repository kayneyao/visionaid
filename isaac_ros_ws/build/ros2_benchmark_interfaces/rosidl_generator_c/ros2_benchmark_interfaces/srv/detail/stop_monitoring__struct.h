// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/StopMonitoring.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/StopMonitoring in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StopMonitoring_Request
{
  uint8_t structure_needs_at_least_one_member;
} ros2_benchmark_interfaces__srv__StopMonitoring_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StopMonitoring_Request.
typedef struct ros2_benchmark_interfaces__srv__StopMonitoring_Request__Sequence
{
  ros2_benchmark_interfaces__srv__StopMonitoring_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StopMonitoring_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'start_timestamps'
// Member 'end_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__struct.h"

/// Struct defined in srv/StopMonitoring in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StopMonitoring_Response
{
  /// A list of message key-timestamp pairs for when the messages are published.
  /// Empty if record_start_timestamps is false.
  ros2_benchmark_interfaces__msg__TimestampedMessageArray start_timestamps;
  /// A list of message key-timestamp pairs for when the messages are received
  ros2_benchmark_interfaces__msg__TimestampedMessageArray end_timestamps;
} ros2_benchmark_interfaces__srv__StopMonitoring_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StopMonitoring_Response.
typedef struct ros2_benchmark_interfaces__srv__StopMonitoring_Response__Sequence
{
  ros2_benchmark_interfaces__srv__StopMonitoring_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StopMonitoring_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__STRUCT_H_
