// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/PlayMessages.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/play_messages__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlayMessages_Request_revise_timestamps_as_message_ids
{
public:
  explicit Init_PlayMessages_Request_revise_timestamps_as_message_ids(::ros2_benchmark_interfaces::srv::PlayMessages_Request & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::PlayMessages_Request revise_timestamps_as_message_ids(::ros2_benchmark_interfaces::srv::PlayMessages_Request::_revise_timestamps_as_message_ids_type arg)
  {
    msg_.revise_timestamps_as_message_ids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Request msg_;
};

class Init_PlayMessages_Request_enforce_publisher_rate
{
public:
  explicit Init_PlayMessages_Request_enforce_publisher_rate(::ros2_benchmark_interfaces::srv::PlayMessages_Request & msg)
  : msg_(msg)
  {}
  Init_PlayMessages_Request_revise_timestamps_as_message_ids enforce_publisher_rate(::ros2_benchmark_interfaces::srv::PlayMessages_Request::_enforce_publisher_rate_type arg)
  {
    msg_.enforce_publisher_rate = std::move(arg);
    return Init_PlayMessages_Request_revise_timestamps_as_message_ids(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Request msg_;
};

class Init_PlayMessages_Request_target_publisher_rate
{
public:
  explicit Init_PlayMessages_Request_target_publisher_rate(::ros2_benchmark_interfaces::srv::PlayMessages_Request & msg)
  : msg_(msg)
  {}
  Init_PlayMessages_Request_enforce_publisher_rate target_publisher_rate(::ros2_benchmark_interfaces::srv::PlayMessages_Request::_target_publisher_rate_type arg)
  {
    msg_.target_publisher_rate = std::move(arg);
    return Init_PlayMessages_Request_enforce_publisher_rate(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Request msg_;
};

class Init_PlayMessages_Request_message_count
{
public:
  explicit Init_PlayMessages_Request_message_count(::ros2_benchmark_interfaces::srv::PlayMessages_Request & msg)
  : msg_(msg)
  {}
  Init_PlayMessages_Request_target_publisher_rate message_count(::ros2_benchmark_interfaces::srv::PlayMessages_Request::_message_count_type arg)
  {
    msg_.message_count = std::move(arg);
    return Init_PlayMessages_Request_target_publisher_rate(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Request msg_;
};

class Init_PlayMessages_Request_playback_mode
{
public:
  Init_PlayMessages_Request_playback_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlayMessages_Request_message_count playback_mode(::ros2_benchmark_interfaces::srv::PlayMessages_Request::_playback_mode_type arg)
  {
    msg_.playback_mode = std::move(arg);
    return Init_PlayMessages_Request_message_count(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::PlayMessages_Request>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_PlayMessages_Request_playback_mode();
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_PlayMessages_Response_timestamps
{
public:
  explicit Init_PlayMessages_Response_timestamps(::ros2_benchmark_interfaces::srv::PlayMessages_Response & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::PlayMessages_Response timestamps(::ros2_benchmark_interfaces::srv::PlayMessages_Response::_timestamps_type arg)
  {
    msg_.timestamps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Response msg_;
};

class Init_PlayMessages_Response_success
{
public:
  Init_PlayMessages_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlayMessages_Response_timestamps success(::ros2_benchmark_interfaces::srv::PlayMessages_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlayMessages_Response_timestamps(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::PlayMessages_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::PlayMessages_Response>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_PlayMessages_Response_success();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__PLAY_MESSAGES__BUILDER_HPP_
