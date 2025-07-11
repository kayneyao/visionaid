// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/GetTopicMessageTimestamps.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/get_topic_message_timestamps__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTopicMessageTimestamps_Request_end_time_offset_ns
{
public:
  explicit Init_GetTopicMessageTimestamps_Request_end_time_offset_ns(::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request end_time_offset_ns(::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request::_end_time_offset_ns_type arg)
  {
    msg_.end_time_offset_ns = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request msg_;
};

class Init_GetTopicMessageTimestamps_Request_start_time_offset_ns
{
public:
  Init_GetTopicMessageTimestamps_Request_start_time_offset_ns()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTopicMessageTimestamps_Request_end_time_offset_ns start_time_offset_ns(::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request::_start_time_offset_ns_type arg)
  {
    msg_.start_time_offset_ns = std::move(arg);
    return Init_GetTopicMessageTimestamps_Request_end_time_offset_ns(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Request>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_GetTopicMessageTimestamps_Request_start_time_offset_ns();
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetTopicMessageTimestamps_Response_success
{
public:
  explicit Init_GetTopicMessageTimestamps_Response_success(::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response success(::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response msg_;
};

class Init_GetTopicMessageTimestamps_Response_topic_message_timestamps
{
public:
  Init_GetTopicMessageTimestamps_Response_topic_message_timestamps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTopicMessageTimestamps_Response_success topic_message_timestamps(::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response::_topic_message_timestamps_type arg)
  {
    msg_.topic_message_timestamps = std::move(arg);
    return Init_GetTopicMessageTimestamps_Response_success(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::GetTopicMessageTimestamps_Response>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_GetTopicMessageTimestamps_Response_topic_message_timestamps();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__GET_TOPIC_MESSAGE_TIMESTAMPS__BUILDER_HPP_
