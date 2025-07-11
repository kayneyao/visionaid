// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros2_benchmark_interfaces:srv/StartRecording.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__STRUCT_H_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__struct.h"

/// Struct defined in srv/StartRecording in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StartRecording_Request
{
  /// This service is used to request a playback node to start recording
  /// incoming messages.
  /// The number of messages to be buffered for each topic.
  /// All incoming messages will be buffered if set to 0.
  uint64_t buffer_length;
  /// The maximum time, in seconds, to wait for all expected number of
  /// messages to be buffereed
  int64_t timeout;
  /// An array that holds the message playback timeline for each topic.
  /// It must be provided if the timeline playback mode is to be used.
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence topic_message_timestamps;
  /// Enable recording message arrival timestamps and overriding
  /// topic_message_timestamps
  bool record_data_timeline;
} ros2_benchmark_interfaces__srv__StartRecording_Request;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StartRecording_Request.
typedef struct ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence
{
  ros2_benchmark_interfaces__srv__StartRecording_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'recorded_topic_message_counts'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__struct.h"

/// Struct defined in srv/StartRecording in the package ros2_benchmark_interfaces.
typedef struct ros2_benchmark_interfaces__srv__StartRecording_Response
{
  /// Whether or not all expected messages were received successfully
  bool success;
  /// The total number of messages being recorded
  uint64_t recorded_message_count;
  /// The number of messages recorded for each topic
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence recorded_topic_message_counts;
} ros2_benchmark_interfaces__srv__StartRecording_Response;

// Struct for a sequence of ros2_benchmark_interfaces__srv__StartRecording_Response.
typedef struct ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence
{
  ros2_benchmark_interfaces__srv__StartRecording_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__STRUCT_H_
