// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:msg/TimestampedMessageArray.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/msg/detail/timestamped_message_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace msg
{

namespace builder
{

class Init_TimestampedMessageArray_timestamps_ns
{
public:
  explicit Init_TimestampedMessageArray_timestamps_ns(::ros2_benchmark_interfaces::msg::TimestampedMessageArray & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::msg::TimestampedMessageArray timestamps_ns(::ros2_benchmark_interfaces::msg::TimestampedMessageArray::_timestamps_ns_type arg)
  {
    msg_.timestamps_ns = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::msg::TimestampedMessageArray msg_;
};

class Init_TimestampedMessageArray_keys
{
public:
  Init_TimestampedMessageArray_keys()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimestampedMessageArray_timestamps_ns keys(::ros2_benchmark_interfaces::msg::TimestampedMessageArray::_keys_type arg)
  {
    msg_.keys = std::move(arg);
    return Init_TimestampedMessageArray_timestamps_ns(msg_);
  }

private:
  ::ros2_benchmark_interfaces::msg::TimestampedMessageArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::msg::TimestampedMessageArray>()
{
  return ros2_benchmark_interfaces::msg::builder::Init_TimestampedMessageArray_keys();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__BUILDER_HPP_
