// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartRecording.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/start_recording__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartRecording_Request_record_data_timeline
{
public:
  explicit Init_StartRecording_Request_record_data_timeline(::ros2_benchmark_interfaces::srv::StartRecording_Request & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::StartRecording_Request record_data_timeline(::ros2_benchmark_interfaces::srv::StartRecording_Request::_record_data_timeline_type arg)
  {
    msg_.record_data_timeline = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Request msg_;
};

class Init_StartRecording_Request_topic_message_timestamps
{
public:
  explicit Init_StartRecording_Request_topic_message_timestamps(::ros2_benchmark_interfaces::srv::StartRecording_Request & msg)
  : msg_(msg)
  {}
  Init_StartRecording_Request_record_data_timeline topic_message_timestamps(::ros2_benchmark_interfaces::srv::StartRecording_Request::_topic_message_timestamps_type arg)
  {
    msg_.topic_message_timestamps = std::move(arg);
    return Init_StartRecording_Request_record_data_timeline(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Request msg_;
};

class Init_StartRecording_Request_timeout
{
public:
  explicit Init_StartRecording_Request_timeout(::ros2_benchmark_interfaces::srv::StartRecording_Request & msg)
  : msg_(msg)
  {}
  Init_StartRecording_Request_topic_message_timestamps timeout(::ros2_benchmark_interfaces::srv::StartRecording_Request::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_StartRecording_Request_topic_message_timestamps(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Request msg_;
};

class Init_StartRecording_Request_buffer_length
{
public:
  Init_StartRecording_Request_buffer_length()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartRecording_Request_timeout buffer_length(::ros2_benchmark_interfaces::srv::StartRecording_Request::_buffer_length_type arg)
  {
    msg_.buffer_length = std::move(arg);
    return Init_StartRecording_Request_timeout(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StartRecording_Request>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_StartRecording_Request_buffer_length();
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartRecording_Response_recorded_topic_message_counts
{
public:
  explicit Init_StartRecording_Response_recorded_topic_message_counts(::ros2_benchmark_interfaces::srv::StartRecording_Response & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::StartRecording_Response recorded_topic_message_counts(::ros2_benchmark_interfaces::srv::StartRecording_Response::_recorded_topic_message_counts_type arg)
  {
    msg_.recorded_topic_message_counts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Response msg_;
};

class Init_StartRecording_Response_recorded_message_count
{
public:
  explicit Init_StartRecording_Response_recorded_message_count(::ros2_benchmark_interfaces::srv::StartRecording_Response & msg)
  : msg_(msg)
  {}
  Init_StartRecording_Response_recorded_topic_message_counts recorded_message_count(::ros2_benchmark_interfaces::srv::StartRecording_Response::_recorded_message_count_type arg)
  {
    msg_.recorded_message_count = std::move(arg);
    return Init_StartRecording_Response_recorded_topic_message_counts(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Response msg_;
};

class Init_StartRecording_Response_success
{
public:
  Init_StartRecording_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartRecording_Response_recorded_message_count success(::ros2_benchmark_interfaces::srv::StartRecording_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_StartRecording_Response_recorded_message_count(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartRecording_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StartRecording_Response>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_StartRecording_Response_success();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_RECORDING__BUILDER_HPP_
