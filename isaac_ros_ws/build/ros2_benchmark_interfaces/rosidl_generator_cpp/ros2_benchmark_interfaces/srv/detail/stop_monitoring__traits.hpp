// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:srv/StopMonitoring.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/srv/detail/stop_monitoring__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StopMonitoring_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopMonitoring_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopMonitoring_Request & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::StopMonitoring_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::StopMonitoring_Request & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StopMonitoring_Request>()
{
  return "ros2_benchmark_interfaces::srv::StopMonitoring_Request";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StopMonitoring_Request>()
{
  return "ros2_benchmark_interfaces/srv/StopMonitoring_Request";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StopMonitoring_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StopMonitoring_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::StopMonitoring_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'start_timestamps'
// Member 'end_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StopMonitoring_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_timestamps
  {
    out << "start_timestamps: ";
    to_flow_style_yaml(msg.start_timestamps, out);
    out << ", ";
  }

  // member: end_timestamps
  {
    out << "end_timestamps: ";
    to_flow_style_yaml(msg.end_timestamps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopMonitoring_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_timestamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_timestamps:\n";
    to_block_style_yaml(msg.start_timestamps, out, indentation + 2);
  }

  // member: end_timestamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_timestamps:\n";
    to_block_style_yaml(msg.end_timestamps, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopMonitoring_Response & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::StopMonitoring_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::StopMonitoring_Response & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StopMonitoring_Response>()
{
  return "ros2_benchmark_interfaces::srv::StopMonitoring_Response";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StopMonitoring_Response>()
{
  return "ros2_benchmark_interfaces/srv/StopMonitoring_Response";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StopMonitoring_Response>
  : std::integral_constant<bool, has_fixed_size<ros2_benchmark_interfaces::msg::TimestampedMessageArray>::value> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StopMonitoring_Response>
  : std::integral_constant<bool, has_bounded_size<ros2_benchmark_interfaces::msg::TimestampedMessageArray>::value> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::StopMonitoring_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StopMonitoring>()
{
  return "ros2_benchmark_interfaces::srv::StopMonitoring";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StopMonitoring>()
{
  return "ros2_benchmark_interfaces/srv/StopMonitoring";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StopMonitoring>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_benchmark_interfaces::srv::StopMonitoring_Request>::value &&
    has_fixed_size<ros2_benchmark_interfaces::srv::StopMonitoring_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StopMonitoring>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_benchmark_interfaces::srv::StopMonitoring_Request>::value &&
    has_bounded_size<ros2_benchmark_interfaces::srv::StopMonitoring_Response>::value
  >
{
};

template<>
struct is_service<ros2_benchmark_interfaces::srv::StopMonitoring>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_benchmark_interfaces::srv::StopMonitoring_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_benchmark_interfaces::srv::StopMonitoring_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_MONITORING__TRAITS_HPP_
