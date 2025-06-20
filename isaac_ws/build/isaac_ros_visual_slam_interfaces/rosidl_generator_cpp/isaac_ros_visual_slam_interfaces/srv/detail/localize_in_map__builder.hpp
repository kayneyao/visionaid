// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from isaac_ros_visual_slam_interfaces:srv/LocalizeInMap.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__BUILDER_HPP_
#define ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "isaac_ros_visual_slam_interfaces/srv/detail/localize_in_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

namespace builder
{

class Init_LocalizeInMap_Request_pose_hint
{
public:
  explicit Init_LocalizeInMap_Request_pose_hint(::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request & msg)
  : msg_(msg)
  {}
  ::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request pose_hint(::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request::_pose_hint_type arg)
  {
    msg_.pose_hint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request msg_;
};

class Init_LocalizeInMap_Request_map_folder_path
{
public:
  Init_LocalizeInMap_Request_map_folder_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalizeInMap_Request_pose_hint map_folder_path(::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request::_map_folder_path_type arg)
  {
    msg_.map_folder_path = std::move(arg);
    return Init_LocalizeInMap_Request_pose_hint(msg_);
  }

private:
  ::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>()
{
  return isaac_ros_visual_slam_interfaces::srv::builder::Init_LocalizeInMap_Request_map_folder_path();
}

}  // namespace isaac_ros_visual_slam_interfaces


namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

namespace builder
{

class Init_LocalizeInMap_Response_success
{
public:
  Init_LocalizeInMap_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response success(::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>()
{
  return isaac_ros_visual_slam_interfaces::srv::builder::Init_LocalizeInMap_Response_success();
}

}  // namespace isaac_ros_visual_slam_interfaces

#endif  // ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__BUILDER_HPP_
