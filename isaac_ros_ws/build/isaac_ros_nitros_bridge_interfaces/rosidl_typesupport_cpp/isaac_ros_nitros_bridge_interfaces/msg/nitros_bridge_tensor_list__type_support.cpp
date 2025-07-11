// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace isaac_ros_nitros_bridge_interfaces
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _NitrosBridgeTensorList_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NitrosBridgeTensorList_type_support_ids_t;

static const _NitrosBridgeTensorList_type_support_ids_t _NitrosBridgeTensorList_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _NitrosBridgeTensorList_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NitrosBridgeTensorList_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NitrosBridgeTensorList_type_support_symbol_names_t _NitrosBridgeTensorList_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, isaac_ros_nitros_bridge_interfaces, msg, NitrosBridgeTensorList)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, isaac_ros_nitros_bridge_interfaces, msg, NitrosBridgeTensorList)),
  }
};

typedef struct _NitrosBridgeTensorList_type_support_data_t
{
  void * data[2];
} _NitrosBridgeTensorList_type_support_data_t;

static _NitrosBridgeTensorList_type_support_data_t _NitrosBridgeTensorList_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NitrosBridgeTensorList_message_typesupport_map = {
  2,
  "isaac_ros_nitros_bridge_interfaces",
  &_NitrosBridgeTensorList_message_typesupport_ids.typesupport_identifier[0],
  &_NitrosBridgeTensorList_message_typesupport_symbol_names.symbol_name[0],
  &_NitrosBridgeTensorList_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NitrosBridgeTensorList_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NitrosBridgeTensorList_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace isaac_ros_nitros_bridge_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>()
{
  return &::isaac_ros_nitros_bridge_interfaces::msg::rosidl_typesupport_cpp::NitrosBridgeTensorList_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, isaac_ros_nitros_bridge_interfaces, msg, NitrosBridgeTensorList)() {
  return get_message_type_support_handle<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeTensorList>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
