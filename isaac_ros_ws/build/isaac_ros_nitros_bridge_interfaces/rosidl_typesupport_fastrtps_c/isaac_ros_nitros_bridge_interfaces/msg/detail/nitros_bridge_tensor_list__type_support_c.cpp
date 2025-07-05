// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from isaac_ros_nitros_bridge_interfaces:msg/NitrosBridgeTensorList.idl
// generated code does not contain a copyright notice
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__struct.h"
#include "isaac_ros_nitros_bridge_interfaces/msg/detail/nitros_bridge_tensor_list__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "isaac_ros_tensor_list_interfaces/msg/detail/tensor__functions.h"  // tensors
#include "rosidl_runtime_c/primitives_sequence.h"  // cuda_event_handle
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // cuda_event_handle
#include "rosidl_runtime_c/string.h"  // uid
#include "rosidl_runtime_c/string_functions.h"  // uid
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_isaac_ros_nitros_bridge_interfaces
size_t get_serialized_size_isaac_ros_tensor_list_interfaces__msg__Tensor(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_isaac_ros_nitros_bridge_interfaces
size_t max_serialized_size_isaac_ros_tensor_list_interfaces__msg__Tensor(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_isaac_ros_nitros_bridge_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, isaac_ros_tensor_list_interfaces, msg, Tensor)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_isaac_ros_nitros_bridge_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_isaac_ros_nitros_bridge_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_isaac_ros_nitros_bridge_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _NitrosBridgeTensorList__ros_msg_type = isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList;

static bool _NitrosBridgeTensorList__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _NitrosBridgeTensorList__ros_msg_type * ros_message = static_cast<const _NitrosBridgeTensorList__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: tensors
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, isaac_ros_tensor_list_interfaces, msg, Tensor
      )()->data);
    size_t size = ros_message->tensors.size;
    auto array_ptr = ros_message->tensors.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: pid
  {
    cdr << ros_message->pid;
  }

  // Field name: fd
  {
    cdr << ros_message->fd;
  }

  // Field name: cuda_event_handle
  {
    size_t size = ros_message->cuda_event_handle.size;
    auto array_ptr = ros_message->cuda_event_handle.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: uid
  {
    const rosidl_runtime_c__String * str = &ros_message->uid;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: device_id
  {
    cdr << ros_message->device_id;
  }

  return true;
}

static bool _NitrosBridgeTensorList__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _NitrosBridgeTensorList__ros_msg_type * ros_message = static_cast<_NitrosBridgeTensorList__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: tensors
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, isaac_ros_tensor_list_interfaces, msg, Tensor
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->tensors.data) {
      isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__fini(&ros_message->tensors);
    }
    if (!isaac_ros_tensor_list_interfaces__msg__Tensor__Sequence__init(&ros_message->tensors, size)) {
      fprintf(stderr, "failed to create array for field 'tensors'");
      return false;
    }
    auto array_ptr = ros_message->tensors.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: pid
  {
    cdr >> ros_message->pid;
  }

  // Field name: fd
  {
    cdr >> ros_message->fd;
  }

  // Field name: cuda_event_handle
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->cuda_event_handle.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->cuda_event_handle);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->cuda_event_handle, size)) {
      fprintf(stderr, "failed to create array for field 'cuda_event_handle'");
      return false;
    }
    auto array_ptr = ros_message->cuda_event_handle.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: uid
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->uid.data) {
      rosidl_runtime_c__String__init(&ros_message->uid);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->uid,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'uid'\n");
      return false;
    }
  }

  // Field name: device_id
  {
    cdr >> ros_message->device_id;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_isaac_ros_nitros_bridge_interfaces
size_t get_serialized_size_isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _NitrosBridgeTensorList__ros_msg_type * ros_message = static_cast<const _NitrosBridgeTensorList__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name tensors
  {
    size_t array_size = ros_message->tensors.size;
    auto array_ptr = ros_message->tensors.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_isaac_ros_tensor_list_interfaces__msg__Tensor(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name pid
  {
    size_t item_size = sizeof(ros_message->pid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fd
  {
    size_t item_size = sizeof(ros_message->fd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cuda_event_handle
  {
    size_t array_size = ros_message->cuda_event_handle.size;
    auto array_ptr = ros_message->cuda_event_handle.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name uid
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->uid.size + 1);
  // field.name device_id
  {
    size_t item_size = sizeof(ros_message->device_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _NitrosBridgeTensorList__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_isaac_ros_nitros_bridge_interfaces
size_t max_serialized_size_isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: tensors
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_isaac_ros_tensor_list_interfaces__msg__Tensor(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: pid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: fd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: cuda_event_handle
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: uid
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: device_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList;
    is_plain =
      (
      offsetof(DataType, device_id) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _NitrosBridgeTensorList__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_isaac_ros_nitros_bridge_interfaces__msg__NitrosBridgeTensorList(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_NitrosBridgeTensorList = {
  "isaac_ros_nitros_bridge_interfaces::msg",
  "NitrosBridgeTensorList",
  _NitrosBridgeTensorList__cdr_serialize,
  _NitrosBridgeTensorList__cdr_deserialize,
  _NitrosBridgeTensorList__get_serialized_size,
  _NitrosBridgeTensorList__max_serialized_size
};

static rosidl_message_type_support_t _NitrosBridgeTensorList__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_NitrosBridgeTensorList,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, isaac_ros_nitros_bridge_interfaces, msg, NitrosBridgeTensorList)() {
  return &_NitrosBridgeTensorList__type_support;
}

#if defined(__cplusplus)
}
#endif
