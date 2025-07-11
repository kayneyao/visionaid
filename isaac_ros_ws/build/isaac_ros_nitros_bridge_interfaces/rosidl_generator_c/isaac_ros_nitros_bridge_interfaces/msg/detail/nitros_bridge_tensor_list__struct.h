// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__STRUCT_H_
#define ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'tensors'
#include "isaac_ros_tensor_list_interfaces/msg/detail/tensor__struct.h"
// Member 'cuda_event_handle'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'uid'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/NitrosBridgeTensorList in the package isaac_ros_nitros_bridge_interfaces.
typedef struct isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList
{
  std_msgs__msg__Header header;
  /// A list of tensors
  isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence tensors;
  int32_t pid;
  int32_t fd;
  rosidl_runtime_c__uint8__Sequence cuda_event_handle;
  rosidl_runtime_c__String uid;
  uint32_t device_id;
} isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList;

// Struct for a sequence of isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList.
typedef struct isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence
{
  isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__STRUCT_H_
