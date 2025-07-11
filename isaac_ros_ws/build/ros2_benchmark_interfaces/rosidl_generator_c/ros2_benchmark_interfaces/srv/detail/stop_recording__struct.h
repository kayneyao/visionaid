// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/StopRecording.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_RECORDING__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_RECORDING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/StopRecording in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StopRecording_Request
{
  uint8_t structure_needs_at_least_one_member;
} ros2_benchmark_interfaces__srv__StopRecording_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StopRecording_Request.
typedef struct ros2_benchmark_interfaces__srv__StopRecording_Request__Sequence
{
  ros2_benchmark_interfaces__srv__StopRecording_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StopRecording_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/StopRecording in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StopRecording_Response
{
  uint8_t structure_needs_at_least_one_member;
} ros2_benchmark_interfaces__srv__StopRecording_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StopRecording_Response.
typedef struct ros2_benchmark_interfaces__srv__StopRecording_Response__Sequence
{
  ros2_benchmark_interfaces__srv__StopRecording_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StopRecording_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_RECORDING__STRUCT_H_
