// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:srv/SetData.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/srv/detail/set_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetData_Request_publish_tf_static_messages
{
public:
  explicit Init_SetData_Request_publish_tf_static_messages(::ros2_benchmark_interfaces::srv::SetData_Request & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::srv::SetData_Request publish_tf_static_messages(::ros2_benchmark_interfaces::srv::SetData_Request::_publish_tf_static_messages_type arg)
  {
    msg_.publish_tf_static_messages = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::SetData_Request msg_;
};

class Init_SetData_Request_publish_tf_messages
{
public:
  explicit Init_SetData_Request_publish_tf_messages(::ros2_benchmark_interfaces::srv::SetData_Request & msg)
  : msg_(msg)
  {}
  Init_SetData_Request_publish_tf_static_messages publish_tf_messages(::ros2_benchmark_interfaces::srv::SetData_Request::_publish_tf_messages_type arg)
  {
    msg_.publish_tf_messages = std::move(arg);
    return Init_SetData_Request_publish_tf_static_messages(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::SetData_Request msg_;
};

class Init_SetData_Request_data_path
{
public:
  Init_SetData_Request_data_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetData_Request_publish_tf_messages data_path(::ros2_benchmark_interfaces::srv::SetData_Request::_data_path_type arg)
  {
    msg_.data_path = std::move(arg);
    return Init_SetData_Request_publish_tf_messages(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::SetData_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::SetData_Request>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_SetData_Request_data_path();
}

}  // namespace ros2_benchmark_interfaces


namespace ros2_benchmark_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetData_Response_success
{
public:
  Init_SetData_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros2_benchmark_interfaces::srv::SetData_Response success(::ros2_benchmark_interfaces::srv::SetData_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::srv::SetData_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::srv::SetData_Response>()
{
  return ros2_benchmark_interfaces::srv::builder::Init_SetData_Response_success();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__BUILDER_HPP_
