// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__BUILDER_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace isaac_ros_nitros_bridge_interfaces
{

namespace msg
{

namespace builder
{

class Init_NitrosBridgeTensorList_device_id
{
public:
  explicit Init_NitrosBridgeTensorList_device_id(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
  : msg_(msg)
  {}
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList device_id(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

class Init_NitrosBridgeTensorList_uid
{
public:
  explicit Init_NitrosBridgeTensorList_uid(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
  : msg_(msg)
  {}
  Init_NitrosBridgeTensorList_device_id uid(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_uid_type arg)
  {
    msg_.uid = std::move(arg);
    return Init_NitrosBridgeTensorList_device_id(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

class Init_NitrosBridgeTensorList_cuda_event_handle
{
public:
  explicit Init_NitrosBridgeTensorList_cuda_event_handle(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
  : msg_(msg)
  {}
  Init_NitrosBridgeTensorList_uid cuda_event_handle(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_cuda_event_handle_type arg)
  {
    msg_.cuda_event_handle = std::move(arg);
    return Init_NitrosBridgeTensorList_uid(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

class Init_NitrosBridgeTensorList_fd
{
public:
  explicit Init_NitrosBridgeTensorList_fd(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
  : msg_(msg)
  {}
  Init_NitrosBridgeTensorList_cuda_event_handle fd(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_fd_type arg)
  {
    msg_.fd = std::move(arg);
    return Init_NitrosBridgeTensorList_cuda_event_handle(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

class Init_NitrosBridgeTensorList_pid
{
public:
  explicit Init_NitrosBridgeTensorList_pid(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
  : msg_(msg)
  {}
  Init_NitrosBridgeTensorList_fd pid(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_pid_type arg)
  {
    msg_.pid = std::move(arg);
    return Init_NitrosBridgeTensorList_fd(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

class Init_NitrosBridgeTensorList_tensors
{
public:
  explicit Init_NitrosBridgeTensorList_tensors(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
  : msg_(msg)
  {}
  Init_NitrosBridgeTensorList_pid tensors(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_tensors_type arg)
  {
    msg_.tensors = std::move(arg);
    return Init_NitrosBridgeTensorList_pid(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

class Init_NitrosBridgeTensorList_header
{
public:
  Init_NitrosBridgeTensorList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NitrosBridgeTensorList_tensors header(::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NitrosBridgeTensorList_tensors(msg_);
  }

private:
  ::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>()
{
  return isaac_ros_nitros_bridge_interfaces::msg::builder::Init_NitrosBridgeTensorList_header();
}

}  // namespace isaac_ros_nitros_bridge_interfaces

#endif  // ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__BUILDER_HPP_
