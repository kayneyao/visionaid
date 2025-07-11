// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/StopLoading.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_LOADING__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_LOADING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/stop_loading__struct.hpp"
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
auto build<::ros2_benchmark_interfaces::srv::StopLoading_Request>()
{
  return ::ros2_benchmark_interfaces::srv::StopLoading_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
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
auto build<::ros2_benchmark_interfaces::srv::StopLoading_Response>()
{
  return ::ros2_benchmark_interfaces::srv::StopLoading_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_LOADING__BUILDER_HPP_
