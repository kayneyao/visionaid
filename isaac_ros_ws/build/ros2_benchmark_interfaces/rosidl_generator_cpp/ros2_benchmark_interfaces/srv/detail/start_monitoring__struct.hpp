// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:srv/StartMonitoring.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Request __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartMonitoring_Request_
{
  using Type = StartMonitoring_Request_<ContainerAllocator>;

  explicit StartMonitoring_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->revise_timestamps_as_message_ids = false;
      this->record_start_timestamps = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->message_count = 0ull;
      this->revise_timestamps_as_message_ids = false;
      this->record_start_timestamps = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_count = 0ull;
    }
  }

  explicit StartMonitoring_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->revise_timestamps_as_message_ids = false;
      this->record_start_timestamps = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->message_count = 0ull;
      this->revise_timestamps_as_message_ids = false;
      this->record_start_timestamps = false;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_count = 0ull;
    }
  }

  // field types and members
  using _message_count_type =
    uint64_t;
  _message_count_type message_count;
  using _revise_timestamps_as_message_ids_type =
    bool;
  _revise_timestamps_as_message_ids_type revise_timestamps_as_message_ids;
  using _record_start_timestamps_type =
    bool;
  _record_start_timestamps_type record_start_timestamps;

  // setters for named parameter idiom
  Type & set__message_count(
    const uint64_t & _arg)
  {
    this->message_count = _arg;
    return *this;
  }
  Type & set__revise_timestamps_as_message_ids(
    const bool & _arg)
  {
    this->revise_timestamps_as_message_ids = _arg;
    return *this;
  }
  Type & set__record_start_timestamps(
    const bool & _arg)
  {
    this->record_start_timestamps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartMonitoring_Request_ & other) const
  {
    if (this->message_count != other.message_count) {
      return false;
    }
    if (this->revise_timestamps_as_message_ids != other.revise_timestamps_as_message_ids) {
      return false;
    }
    if (this->record_start_timestamps != other.record_start_timestamps) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartMonitoring_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartMonitoring_Request_

// alias to use template instance with default allocator
using StartMonitoring_Request =
  ros2_benchmark_interfaces::srv::StartMonitoring_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Response __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartMonitoring_Response_
{
  using Type = StartMonitoring_Response_<ContainerAllocator>;

  explicit StartMonitoring_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit StartMonitoring_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StartMonitoring_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StartMonitoring_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartMonitoring_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartMonitoring_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartMonitoring_Response_

// alias to use template instance with default allocator
using StartMonitoring_Response =
  ros2_benchmark_interfaces::srv::StartMonitoring_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace ros2_benchmark_interfaces
{

namespace srv
{

struct StartMonitoring
{
  using Request = ros2_benchmark_interfaces::srv::StartMonitoring_Request;
  using Response = ros2_benchmark_interfaces::srv::StartMonitoring_Response;
};

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__START_MONITORING__STRUCT_HPP_
