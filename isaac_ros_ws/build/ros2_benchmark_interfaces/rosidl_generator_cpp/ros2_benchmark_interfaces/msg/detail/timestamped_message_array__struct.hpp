// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:msg/TimestampedMessageArray.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__msg__TimestampedMessageArray __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__msg__TimestampedMessageArray __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TimestampedMessageArray_
{
  using Type = TimestampedMessageArray_<ContainerAllocator>;

  explicit TimestampedMessageArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit TimestampedMessageArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _keys_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _keys_type keys;
  using _timestamps_ns_type =
    std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>>;
  _timestamps_ns_type timestamps_ns;

  // setters for named parameter idiom
  Type & set__keys(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->keys = _arg;
    return *this;
  }
  Type & set__timestamps_ns(
    const std::vector<int64_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int64_t>> & _arg)
  {
    this->timestamps_ns = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__msg__TimestampedMessageArray
    std::shared_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__msg__TimestampedMessageArray
    std::shared_ptr<ros2_benchmark_interfaces::msg::TimestampedMessageArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TimestampedMessageArray_ & other) const
  {
    if (this->keys != other.keys) {
      return false;
    }
    if (this->timestamps_ns != other.timestamps_ns) {
      return false;
    }
    return true;
  }
  bool operator!=(const TimestampedMessageArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TimestampedMessageArray_

// alias to use template instance with default allocator
using TimestampedMessageArray =
  ros2_benchmark_interfaces::msg::TimestampedMessageArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__MSG__DETAIL__TIMESTAMPED_MESSAGE_ARRAY__STRUCT_HPP_
