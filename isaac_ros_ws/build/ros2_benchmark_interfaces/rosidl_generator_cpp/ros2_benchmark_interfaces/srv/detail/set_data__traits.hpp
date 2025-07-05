// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:srv/SetData.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/srv/detail/set_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetData_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: data_path
  {
    out << "data_path: ";
    rosidl_generator_traits::value_to_yaml(msg.data_path, out);
    out << ", ";
  }

  // member: publish_tf_messages
  {
    out << "publish_tf_messages: ";
    rosidl_generator_traits::value_to_yaml(msg.publish_tf_messages, out);
    out << ", ";
  }

  // member: publish_tf_static_messages
  {
    out << "publish_tf_static_messages: ";
    rosidl_generator_traits::value_to_yaml(msg.publish_tf_static_messages, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_path: ";
    rosidl_generator_traits::value_to_yaml(msg.data_path, out);
    out << "\n";
  }

  // member: publish_tf_messages
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "publish_tf_messages: ";
    rosidl_generator_traits::value_to_yaml(msg.publish_tf_messages, out);
    out << "\n";
  }

  // member: publish_tf_static_messages
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "publish_tf_static_messages: ";
    rosidl_generator_traits::value_to_yaml(msg.publish_tf_static_messages, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetData_Request & msg, bool use_flow_style = false)
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

}  // namespace ros2_benchmark_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2_benchmark_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_benchmark_interfaces::srv::SetData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::SetData_Request & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::SetData_Request>()
{
  return "ros2_benchmark_interfaces::srv::SetData_Request";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::SetData_Request>()
{
  return "ros2_benchmark_interfaces/srv/SetData_Request";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::SetData_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::SetData_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::SetData_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetData_Response & msg,
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
  const SetData_Response & msg,
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

inline std::string to_yaml(const SetData_Response & msg, bool use_flow_style = false)
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

}  // namespace ros2_benchmark_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros2_benchmark_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros2_benchmark_interfaces::srv::SetData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::SetData_Response & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::SetData_Response>()
{
  return "ros2_benchmark_interfaces::srv::SetData_Response";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::SetData_Response>()
{
  return "ros2_benchmark_interfaces/srv/SetData_Response";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::SetData_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::SetData_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::SetData_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::SetData>()
{
  return "ros2_benchmark_interfaces::srv::SetData";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::SetData>()
{
  return "ros2_benchmark_interfaces/srv/SetData";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::SetData>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_benchmark_interfaces::srv::SetData_Request>::value &&
    has_fixed_size<ros2_benchmark_interfaces::srv::SetData_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::SetData>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_benchmark_interfaces::srv::SetData_Request>::value &&
    has_bounded_size<ros2_benchmark_interfaces::srv::SetData_Response>::value
  >
{
};

template<>
struct is_service<ros2_benchmark_interfaces::srv::SetData>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_benchmark_interfaces::srv::SetData_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_benchmark_interfaces::srv::SetData_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__SET_DATA__TRAITS_HPP_
