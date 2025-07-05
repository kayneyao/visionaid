// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/StopMonitoring.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/stop_monitoring__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StopMonitoring_Request>()
{
  return ::ros2_benchmark_interfaces::srv::StopMonitoring_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_StopMonitoring_Response_end_timestamps
{
public:
  explicit Init_StopMonitoring_Response_end_timestamps(::ros2_benchmark_interfaces::srv::StopMonitoring_Response & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::StopMonitoring_Response end_timestamps(::ros2_benchmark_interfaces::srv::StopMonitoring_Response::_end_timestamps_type arg)
  {
    msg_.end_timestamps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StopMonitoring_Response msg_;
};

class Init_StopMonitoring_Response_start_timestamps
{
public:
  Init_StopMonitoring_Response_start_timestamps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StopMonitoring_Response_end_timestamps start_timestamps(::ros2_benchmark_interfaces::srv::StopMonitoring_Response::_start_timestamps_type arg)
  {
    msg_.start_timestamps = std::move(arg);
    return Init_StopMonitoring_Response_end_timestamps(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::StopMonitoring_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::StopMonitoring_Response>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_StopMonitoring_Response_start_timestamps();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__BUILDER_HPP_
