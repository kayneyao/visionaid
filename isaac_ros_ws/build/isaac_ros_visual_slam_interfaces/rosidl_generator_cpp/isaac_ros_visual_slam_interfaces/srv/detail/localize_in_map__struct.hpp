// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from isaac_ros_visual_slam_interfaces:srv/LocalizeInMap.idl
// generated code does not contain a copyright notice

#ifndef ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__STRUCT_HPP_
#define ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose_hint'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Request __declspec(deprecated)
#endif

namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LocalizeInMap_Request_
{
  using Type = LocalizeInMap_Request_<ContainerAllocator>;

  explicit LocalizeInMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose_hint(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_folder_path = "";
    }
  }

  explicit LocalizeInMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_folder_path(_alloc),
    pose_hint(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_folder_path = "";
    }
  }

  // field types and members
  using _map_folder_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_folder_path_type map_folder_path;
  using _pose_hint_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_hint_type pose_hint;

  // setters for named parameter idiom
  Type & set__map_folder_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_folder_path = _arg;
    return *this;
  }
  Type & set__pose_hint(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose_hint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Request
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Request
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalizeInMap_Request_ & other) const
  {
    if (this->map_folder_path != other.map_folder_path) {
      return false;
    }
    if (this->pose_hint != other.pose_hint) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalizeInMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalizeInMap_Request_

// alias to use template instance with default allocator
using LocalizeInMap_Request =
  isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace isaac_ros_visual_slam_interfaces


#ifndef _WIN32
# define DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Response __declspec(deprecated)
#endif

namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LocalizeInMap_Response_
{
  using Type = LocalizeInMap_Response_<ContainerAllocator>;

  explicit LocalizeInMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit LocalizeInMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Response
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__isaac_ros_visual_slam_interfaces__srv__LocalizeInMap_Response
    std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalizeInMap_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalizeInMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalizeInMap_Response_

// alias to use template instance with default allocator
using LocalizeInMap_Response =
  isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace isaac_ros_visual_slam_interfaces

namespace isaac_ros_visual_slam_interfaces
{

namespace srv
{

struct LocalizeInMap
{
  using Request = isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Request;
  using Response = isaac_ros_visual_slam_interfaces::srv::LocalizeInMap_Response;
};

}  // namespace srv

}  // namespace isaac_ros_visual_slam_interfaces

#endif  // ISAAC_ROS_VISUAL_SLAM_INTERFACES__SRV__DETAIL__LOCALIZE_IN_MAP__STRUCT_HPP_
