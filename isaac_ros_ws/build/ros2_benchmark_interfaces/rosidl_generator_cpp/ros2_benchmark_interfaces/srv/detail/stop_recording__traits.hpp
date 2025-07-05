// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros2_benchmark_interfaces:srv/StopRecording.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_RECORDING__TRAITS_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_RECORDING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros2_benchmark_interfaces/srv/detail/stop_recording__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StopRecording_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopRecording_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopRecording_Request & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::StopRecording_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::StopRecording_Request & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StopRecording_Request>()
{
  return "ros2_benchmark_interfaces::srv::StopRecording_Request";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StopRecording_Request>()
{
  return "ros2_benchmark_interfaces/srv/StopRecording_Request";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StopRecording_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StopRecording_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::StopRecording_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ros2_benchmark_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const StopRecording_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StopRecording_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StopRecording_Response & msg, bool use_flow_style = false)
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
  const ros2_benchmark_interfaces::srv::StopRecording_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros2_benchmark_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros2_benchmark_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ros2_benchmark_interfaces::srv::StopRecording_Response & msg)
{
  return ros2_benchmark_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StopRecording_Response>()
{
  return "ros2_benchmark_interfaces::srv::StopRecording_Response";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StopRecording_Response>()
{
  return "ros2_benchmark_interfaces/srv/StopRecording_Response";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StopRecording_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StopRecording_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros2_benchmark_interfaces::srv::StopRecording_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ros2_benchmark_interfaces::srv::StopRecording>()
{
  return "ros2_benchmark_interfaces::srv::StopRecording";
}

template<>
inline const char * name<ros2_benchmark_interfaces::srv::StopRecording>()
{
  return "ros2_benchmark_interfaces/srv/StopRecording";
}

template<>
struct has_fixed_size<ros2_benchmark_interfaces::srv::StopRecording>
  : std::integral_constant<
    bool,
    has_fixed_size<ros2_benchmark_interfaces::srv::StopRecording_Request>::value &&
    has_fixed_size<ros2_benchmark_interfaces::srv::StopRecording_Response>::value
  >
{
};

template<>
struct has_bounded_size<ros2_benchmark_interfaces::srv::StopRecording>
  : std::integral_constant<
    bool,
    has_bounded_size<ros2_benchmark_interfaces::srv::StopRecording_Request>::value &&
    has_bounded_size<ros2_benchmark_interfaces::srv::StopRecording_Response>::value
  >
{
};

template<>
struct is_service<ros2_benchmark_interfaces::srv::StopRecording>
  : std::true_type
{
};

template<>
struct is_service_request<ros2_benchmark_interfaces::srv::StopRecording_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ros2_benchmark_interfaces::srv::StopRecording_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_RECORDING__TRAITS_HPP_
