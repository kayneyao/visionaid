// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartLoading.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/start_loading__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartLoading_Request_publish_in_real_time
{
public:
  explicit Init_StartLoading_Request_publish_in_real_time(::ros2_benchmark_interfaces::srv::StartLoading_Request & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::StartLoading_Request publish_in_real_time(::ros2_benchmark_interfaces::srv::StartLoading_Request::_publish_in_real_time_type arg)
  {
    msg_.publish_in_real_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Request msg_;
};

class Init_StartLoading_Request_repeat_data
{
public:
  explicit Init_StartLoading_Request_repeat_data(::ros2_benchmark_interfaces::srv::StartLoading_Request & msg)
  : msg_(msg)
  {}
  Init_StartLoading_Request_publish_in_real_time repeat_data(::ros2_benchmark_interfaces::srv::StartLoading_Request::_repeat_data_type arg)
  {
    msg_.repeat_data = std::move(arg);
    return Init_StartLoading_Request_publish_in_real_time(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Request msg_;
};

class Init_StartLoading_Request_end_time_offset_ns
{
public:
  explicit Init_StartLoading_Request_end_time_offset_ns(::ros2_benchmark_interfaces::srv::StartLoading_Request & msg)
  : msg_(msg)
  {}
  Init_StartLoading_Request_repeat_data end_time_offset_ns(::ros2_benchmark_interfaces::srv::StartLoading_Request::_end_time_offset_ns_type arg)
  {
    msg_.end_time_offset_ns = std::move(arg);
    return Init_StartLoading_Request_repeat_data(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Request msg_;
};

class Init_StartLoading_Request_start_time_offset_ns
{
public:
  Init_StartLoading_Request_start_time_offset_ns()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartLoading_Request_end_time_offset_ns start_time_offset_ns(::ros2_benchmark_interfaces::srv::StartLoading_Request::_start_time_offset_ns_type arg)
  {
    msg_.start_time_offset_ns = std::move(arg);
    return Init_StartLoading_Request_end_time_offset_ns(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StartLoading_Request>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_StartLoading_Request_start_time_offset_ns();
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartLoading_Response_success
{
public:
  explicit Init_StartLoading_Response_success(::ros2_benchmark_interfaces::srv::StartLoading_Response & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::StartLoading_Response success(::ros2_benchmark_interfaces::srv::StartLoading_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Response msg_;
};

class Init_StartLoading_Response_played_message_count
{
public:
  explicit Init_StartLoading_Response_played_message_count(::ros2_benchmark_interfaces::srv::StartLoading_Response & msg)
  : msg_(msg)
  {}
  Init_StartLoading_Response_success played_message_count(::ros2_benchmark_interfaces::srv::StartLoading_Response::_played_message_count_type arg)
  {
    msg_.played_message_count = std::move(arg);
    return Init_StartLoading_Response_success(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Response msg_;
};

class Init_StartLoading_Response_topic_message_timestamps
{
public:
  Init_StartLoading_Response_topic_message_timestamps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartLoading_Response_played_message_count topic_message_timestamps(::ros2_benchmark_interfaces::srv::StartLoading_Response::_topic_message_timestamps_type arg)
  {
    msg_.topic_message_timestamps = std::move(arg);
    return Init_StartLoading_Response_played_message_count(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartLoading_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StartLoading_Response>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_StartLoading_Response_topic_message_timestamps();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__BUILDER_HPP_
