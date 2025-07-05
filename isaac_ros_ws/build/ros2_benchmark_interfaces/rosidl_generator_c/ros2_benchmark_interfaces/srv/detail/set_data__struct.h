// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/SetData.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetData in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__SetData_Request
{
  /// This service sets the path for the data file to be loaded in a data
  /// loader node.
  /// Path to the data to be loaded
  rosidl_runtime_c__String data_path;
  /// Whether to publish /tf and /tf_static messages when setting up data
  bool publish_tf_messages;
  bool publish_tf_static_messages;
} ros2_benchmark_interfaces__srv__SetData_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__SetData_Request.
typedef struct ros2_benchmark_interfaces__srv__SetData_Request__Sequence
{
  ros2_benchmark_interfaces__srv__SetData_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__SetData_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetData in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__SetData_Response
{
  /// True if no error occured
  bool success;
} ros2_benchmark_interfaces__srv__SetData_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__SetData_Response.
typedef struct ros2_benchmark_interfaces__srv__SetData_Response__Sequence
{
  ros2_benchmark_interfaces__srv__SetData_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__SetData_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__STRUCT_H_
