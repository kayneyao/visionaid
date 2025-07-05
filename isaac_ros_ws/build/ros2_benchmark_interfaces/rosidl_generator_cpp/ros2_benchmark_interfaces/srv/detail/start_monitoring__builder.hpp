// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartMonitoring.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/start_monitoring__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartMonitoring_Request_record_start_timestamps
{
public:
  explicit Init_StartMonitoring_Request_record_start_timestamps(::ros2_benchmark_interfaces::srv::StartMonitoring_Request & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::StartMonitoring_Request record_start_timestamps(::ros2_benchmark_interfaces::srv::StartMonitoring_Request::_record_start_timestamps_type arg)
  {
    msg_.record_start_timestamps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartMonitoring_Request msg_;
};

class Init_StartMonitoring_Request_revise_timestamps_as_message_ids
{
public:
  explicit Init_StartMonitoring_Request_revise_timestamps_as_message_ids(::ros2_benchmark_interfaces::srv::StartMonitoring_Request & msg)
  : msg_(msg)
  {}
  Init_StartMonitoring_Request_record_start_timestamps revise_timestamps_as_message_ids(::ros2_benchmark_interfaces::srv::StartMonitoring_Request::_revise_timestamps_as_message_ids_type arg)
  {
    msg_.revise_timestamps_as_message_ids = std::move(arg);
    return Init_StartMonitoring_Request_record_start_timestamps(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartMonitoring_Request msg_;
};

class Init_StartMonitoring_Request_message_count
{
public:
  Init_StartMonitoring_Request_message_count()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartMonitoring_Request_revise_timestamps_as_message_ids message_count(::ros2_benchmark_interfaces::srv::StartMonitoring_Request::_message_count_type arg)
  {
    msg_.message_count = std::move(arg);
    return Init_StartMonitoring_Request_revise_timestamps_as_message_ids(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StartMonitoring_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StartMonitoring_Request>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_StartMonitoring_Request_message_count();
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StartMonitoring_Response>()
{
  return ::ros2_benchmark_interfaces::srv::StartMonitoring_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__BUILDER_HPP_
