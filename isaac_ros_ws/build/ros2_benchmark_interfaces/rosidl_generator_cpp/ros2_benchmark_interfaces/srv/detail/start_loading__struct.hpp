// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartLoading.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Request __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartLoading_Request_
{
  using Type = StartLoading_Request_<ContainerAllocator>;

  explicit StartLoading_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->start_time_offset_ns = -1ll;
      this->end_time_offset_ns = -1ll;
      this->repeat_data = true;
      this->publish_in_real_time = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->start_time_offset_ns = 0ll;
      this->end_time_offset_ns = 0ll;
      this->repeat_data = false;
      this->publish_in_real_time = false;
    }
  }

  explicit StartLoading_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->start_time_offset_ns = -1ll;
      this->end_time_offset_ns = -1ll;
      this->repeat_data = true;
      this->publish_in_real_time = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->start_time_offset_ns = 0ll;
      this->end_time_offset_ns = 0ll;
      this->repeat_data = false;
      this->publish_in_real_time = false;
    }
  }

  // field types and members
  using _start_time_offset_ns_type =
    int64_t;
  _start_time_offset_ns_type start_time_offset_ns;
  using _end_time_offset_ns_type =
    int64_t;
  _end_time_offset_ns_type end_time_offset_ns;
  using _repeat_data_type =
    bool;
  _repeat_data_type repeat_data;
  using _publish_in_real_time_type =
    bool;
  _publish_in_real_time_type publish_in_real_time;

  // setters for named parameter idiom
  Type & set__start_time_offset_ns(
    const int64_t & _arg)
  {
    this->start_time_offset_ns = _arg;
    return *this;
  }
  Type & set__end_time_offset_ns(
    const int64_t & _arg)
  {
    this->end_time_offset_ns = _arg;
    return *this;
  }
  Type & set__repeat_data(
    const bool & _arg)
  {
    this->repeat_data = _arg;
    return *this;
  }
  Type & set__publish_in_real_time(
    const bool & _arg)
  {
    this->publish_in_real_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartLoading_Request_ & other) const
  {
    if (this->start_time_offset_ns != other.start_time_offset_ns) {
      return false;
    }
    if (this->end_time_offset_ns != other.end_time_offset_ns) {
      return false;
    }
    if (this->repeat_data != other.repeat_data) {
      return false;
    }
    if (this->publish_in_real_time != other.publish_in_real_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartLoading_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartLoading_Request_

// alias to use template instance with default allocator
using StartLoading_Request =
  ros2_benchmark_interfaces::srv::StartLoading_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces


// Include directives for member types
// Member 'topic_message_timestamps'
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Response __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartLoading_Response_
{
  using Type = StartLoading_Response_<ContainerAllocator>;

  explicit StartLoading_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->played_message_count = 0ull;
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->played_message_count = 0ull;
      this->success = false;
    }
  }

  explicit StartLoading_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->played_message_count = 0ull;
      this->success = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->played_message_count = 0ull;
      this->success = false;
    }
  }

  // field types and members
  using _topic_message_timestamps_type =
    std::vector<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>>>;
  _topic_message_timestamps_type topic_message_timestamps;
  using _played_message_count_type =
    uint64_t;
  _played_message_count_type played_message_count;
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__topic_message_timestamps(
    const std::vector<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros2_benchmark_interfaces::msg::TopicMessageTimestampArray_<ContainerAllocator>>> & _arg)
  {
    this->topic_message_timestamps = _arg;
    return *this;
  }
  Type & set__played_message_count(
    const uint64_t & _arg)
  {
    this->played_message_count = _arg;
    return *this;
  }
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartLoading_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartLoading_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartLoading_Response_ & other) const
  {
    if (this->topic_message_timestamps != other.topic_message_timestamps) {
      return false;
    }
    if (this->played_message_count != other.played_message_count) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartLoading_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartLoading_Response_

// alias to use template instance with default allocator
using StartLoading_Response =
  ros2_benchmark_interfaces::srv::StartLoading_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace ros2_benchmark_interfaces
{

namespace srv
{

struct StartLoading
{
  using Request = ros2_benchmark_interfaces::srv::StartLoading_Request;
  using Response = ros2_benchmark_interfaces::srv::StartLoading_Response;
};

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_LOADING__STRUCT_HPP_
