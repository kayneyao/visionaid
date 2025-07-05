// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros2_benchmark_interfaces:srv/StopLoading.idl
// generated code does not contain a copyright notice

#ifndef ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_LOADING__STRUCT_HPP_
#define ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_LOADING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Request __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StopLoading_Request_
{
  using Type = StopLoading_Request_<ContainerAllocator>;

  explicit StopLoading_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit StopLoading_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Request
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StopLoading_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const StopLoading_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StopLoading_Request_

// alias to use template instance with default allocator
using StopLoading_Request =
  ros2_benchmark_interfaces::srv::StopLoading_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces


#ifndef _WIN32
# define DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Response __declspec(deprecated)
#endif

namespace ros2_benchmark_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StopLoading_Response_
{
  using Type = StopLoading_Response_<ContainerAllocator>;

  explicit StopLoading_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit StopLoading_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros2_benchmark_interfaces__srv__StopLoading_Response
    std::shared_ptr<ros2_benchmark_interfaces::srv::StopLoading_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StopLoading_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const StopLoading_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StopLoading_Response_

// alias to use template instance with default allocator
using StopLoading_Response =
  ros2_benchmark_interfaces::srv::StopLoading_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

namespace ros2_benchmark_interfaces
{

namespace srv
{

struct StopLoading
{
  using Request = ros2_benchmark_interfaces::srv::StopLoading_Request;
  using Response = ros2_benchmark_interfaces::srv::StopLoading_Response;
};

}  // namespace srv

}  // namespace ros2_benchmark_interfaces

#endif  // ROS2_BENCHMARK_INTERFACES__SRV__DETAIL__STOP_LOADING__STRUCT_HPP_
