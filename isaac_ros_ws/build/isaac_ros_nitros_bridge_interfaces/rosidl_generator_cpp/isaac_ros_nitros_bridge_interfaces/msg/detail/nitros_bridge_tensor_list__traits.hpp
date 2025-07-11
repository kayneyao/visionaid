// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__TRAITS_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'tensors'
#include "isaac_ros_tensor_list_interfaces/msg/detail/tensor__traits.hpp"

namespace isaac_ros_nitros_bridge_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NitrosBridgeTensorList & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: tensors
  {
    if (msg.tensors.size() == 0) {
      out << "tensors: []";
    } else {
      out << "tensors: [";
      size_t pending_items = msg.tensors.size();
      for (auto item : msg.tensors) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: pid
  {
    out << "pid: ";
    rosidl_generator_traits::value_to_yaml(msg.pid, out);
    out << ", ";
  }

  // member: fd
  {
    out << "fd: ";
    rosidl_generator_traits::value_to_yaml(msg.fd, out);
    out << ", ";
  }

  // member: cuda_event_handle
  {
    if (msg.cuda_event_handle.size() == 0) {
      out << "cuda_event_handle: []";
    } else {
      out << "cuda_event_handle: [";
      size_t pending_items = msg.cuda_event_handle.size();
      for (auto item : msg.cuda_event_handle) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: uid
  {
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
    out << ", ";
  }

  // member: device_id
  {
    out << "device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.device_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NitrosBridgeTensorList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: tensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tensors.size() == 0) {
      out << "tensors: []\n";
    } else {
      out << "tensors:\n";
      for (auto item : msg.tensors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: pid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pid: ";
    rosidl_generator_traits::value_to_yaml(msg.pid, out);
    out << "\n";
  }

  // member: fd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fd: ";
    rosidl_generator_traits::value_to_yaml(msg.fd, out);
    out << "\n";
  }

  // member: cuda_event_handle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cuda_event_handle.size() == 0) {
      out << "cuda_event_handle: []\n";
    } else {
      out << "cuda_event_handle:\n";
      for (auto item : msg.cuda_event_handle) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: uid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
    out << "\n";
  }

  // member: device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.device_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NitrosBridgeTensorList & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace isaac_ros_nitros_bridge_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use isaac_ros_nitros_bridge_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros_nitros_bridge_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros_nitros_bridge_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList & msg)
{
  return isaac_ros_nitros_bridge_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>()
{
  return "isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList";
}

template<>
inline const char * name<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>()
{
  return "isaac_ros_nitros_bridge_interfaces/msg/NitrosBridgeTensorList";
}

template<>
struct has_fixed_size<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ISAAC_ROS_NITROS_BRIDGE_INTERFACES__MSG__DETAIL__NITROS_BRIDGE_TENSOR_LIST__TRAITS_HPP_
