// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from isaac_ros_visual_slam_interfaces:srv/LocalizeInMap.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__TRAITS_HPP_
#define ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "isaac_ros_visual_slam_interfaces/srv/detail/localize_in_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose_hint'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LocalizeInMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: map_folder_path
  {
    out << "map_folder_path: ";
    rosidl_generator_traits::value_to_yaml(msg.map_folder_path, out);
    out << ", ";
  }

  // member: pose_hint
  {
    out << "pose_hint: ";
    to_flow_style_yaml(msg.pose_hint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalizeInMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_folder_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_folder_path: ";
    rosidl_generator_traits::value_to_yaml(msg.map_folder_path, out);
    out << "\n";
  }

  // member: pose_hint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_hint:\n";
    to_block_style_yaml(msg.pose_hint, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalizeInMap_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace isaac_ros_visual_slam_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use isaac_ros_visual_slam_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros_visual_slam_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros_visual_slam_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request & msg)
{
  return isaac_ros_visual_slam_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>()
{
  return "isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request";
}

template<>
inline const char * name<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>()
{
  return "isaac_ros_visual_slam_interfaces/srv/LocalizeInMap_Request";
}

template<>
struct has_fixed_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const LocalizeInMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalizeInMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalizeInMap_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace isaac_ros_visual_slam_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use isaac_ros_visual_slam_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  isaac_ros_visual_slam_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use isaac_ros_visual_slam_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response & msg)
{
  return isaac_ros_visual_slam_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>()
{
  return "isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response";
}

template<>
inline const char * name<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>()
{
  return "isaac_ros_visual_slam_interfaces/srv/LocalizeInMap_Response";
}

template<>
struct has_fixed_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>()
{
  return "isaac_ros_visual_slam_interfaces::srv::LocalizeInMap";
}

template<>
inline const char * name<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>()
{
  return "isaac_ros_visual_slam_interfaces/srv/LocalizeInMap";
}

template<>
struct has_fixed_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>
  : std::integral_constant<
    bool,
    has_fixed_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>::value &&
    has_fixed_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>
  : std::integral_constant<
    bool,
    has_bounded_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>::value &&
    has_bounded_size<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>::value
  >
{
};

template<>
struct is_service<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap>
  : std::true_type
{
};

template<>
struct is_service_request<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__TRAITS_HPP_
