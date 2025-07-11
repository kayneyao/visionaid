// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros2_benchmark_interfaces:msg/TopicMessageTimestampArray.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_TIMESTAMP_ARRAY__BUILDER_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_TIMESTAMP_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros2_benchmark_interfaces
{

namespace msg
{

namespace builder
{

class Init_TopicMessageTimestampArray_timestamps_ns
{
public:
  explicit Init_TopicMessageTimestampArray_timestamps_ns(::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray & msg)
  : msg_(msg)
  {}
  ::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray timestamps_ns(::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray::_timestamps_ns_type arg)
  {
    msg_.timestamps_ns = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray msg_;
};

class Init_TopicMessageTimestampArray_topic_name
{
public:
  Init_TopicMessageTimestampArray_topic_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TopicMessageTimestampArray_timestamps_ns topic_name(::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return Init_TopicMessageTimestampArray_timestamps_ns(msg_);
  }

private:
  ::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros2_benchmark_interfaces::msg::TopicMessageTimestampArray>()
{
  return ros2_benchmark_interfaces::msg::builder::Init_TopicMessageTimestampArray_topic_name();
}

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TOPIC_MESSAGE_TIMESTAMP_ARRAY__BUILDER_HPP_
