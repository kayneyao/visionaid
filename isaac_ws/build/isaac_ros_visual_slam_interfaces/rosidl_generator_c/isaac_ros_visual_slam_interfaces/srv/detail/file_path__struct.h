// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from isaac_ros_visual_slam_interfaces:srv/FilePath.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__FILE_PATH__STRUCT_H_
#define ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__FILE_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'file_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FilePath in the package isaac_ros_visual_slam_interfaces.
typedef struct isaac_ros_visual_slam_interfaces__srv__FilePath_Request
{
  /// Request
  rosidl_runtime_c__String file_path;
} isaac_ros_visual_slam_interfaces__srv__FilePath_Request;

// Struct for a sequence of isaac_ros_visual_slam_interfaces__srv__FilePath_Request.
typedef struct isaac_ros_visual_slam_interfaces__srv__FilePath_Request__Sequence
{
  isaac_ros_visual_slam_interfaces__srv__FilePath_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros_visual_slam_interfaces__srv__FilePath_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FilePath in the package isaac_ros_visual_slam_interfaces.
typedef struct isaac_ros_visual_slam_interfaces__srv__FilePath_Response
{
  bool success;
} isaac_ros_visual_slam_interfaces__srv__FilePath_Response;

// Struct for a sequence of isaac_ros_visual_slam_interfaces__srv__FilePath_Response.
typedef struct isaac_ros_visual_slam_interfaces__srv__FilePath_Response__Sequence
{
  isaac_ros_visual_slam_interfaces__srv__FilePath_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros_visual_slam_interfaces__srv__FilePath_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__FILE_PATH__STRUCT_H_
