// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/GetTopicMessageTimestamps.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetTopicMessageTimestamps in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Request
{
  /// This service requests a data loader node to provide a timeline for
  /// when the messages load from a rosbag should be played for each topic.
  /// Start time offset (in nanoseconds) of the data to be loaded
  int64_t start_time_offset_ns;
  /// End time offset (in nanoseconds) of the data to be loaded
  int64_t end_time_offset_ns;
} ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Request.
typedef struct ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Request__Sequence
{
  ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__struct.h"

/// Struct defined in srv/GetTopicMessageTimestamps in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Response
{
  /// An array that holds the message playback timeline for each of the
  /// loaded topics
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence topic_message_timestamps;
  /// True if no error occured
  bool success;
} ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Response.
typedef struct ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Response__Sequence
{
  ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__GetTopicMessageTimestamps_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__STRUCT_H_
