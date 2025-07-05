// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/StartLoading.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/StartLoading in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StartLoading_Request
{
  /// This service requests a data loader node to start loading and
  /// publishing messages. It must be called after SetData.srv is called
  /// and finished. The service returns when receiving a StopLoading.srv
  /// service call.
  /// Start time offset (in nanoseconds) of the data to be loaded
  int64_t start_time_offset_ns;
  /// End time offset (in nanoseconds) of the data to be loaded
  int64_t end_time_offset_ns;
  /// Whether to repeat loading data when the end is reached
  bool repeat_data;
  /// Whether to publish messages in real-time based on rosbag timestamps
  bool publish_in_real_time;
} ros2_benchmark_interfaces__srv__StartLoading_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StartLoading_Request.
typedef struct ros2_benchmark_interfaces__srv__StartLoading_Request__Sequence
{
  ros2_benchmark_interfaces__srv__StartLoading_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StartLoading_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__struct.h"

/// Struct defined in srv/StartLoading in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StartLoading_Response
{
  /// An array that holds the message playback timeline for each of
  /// the loaded topics
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence topic_message_timestamps;
  /// The number of messages being sent
  uint64_t played_message_count;
  /// True if no error occured before the service stopped
  bool success;
} ros2_benchmark_interfaces__srv__StartLoading_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StartLoading_Response.
typedef struct ros2_benchmark_interfaces__srv__StartLoading_Response__Sequence
{
  ros2_benchmark_interfaces__srv__StartLoading_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StartLoading_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__STRUCT_H_
