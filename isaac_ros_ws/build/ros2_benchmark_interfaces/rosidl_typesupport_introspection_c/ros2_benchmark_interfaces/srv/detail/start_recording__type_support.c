// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros2_benchmark_interfaces:srv/StartRecording.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros2_benchmark_interfaces/srv/detail/start_recording__rosidl_typesupport_introspection_c.h"
#include "ros2_benchmark_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros2_benchmark_interfaces/srv/detail/start_recording__functions.h"
#include "ros2_benchmark_interfaces/srv/detail/start_recording__struct.h"


// Include directives for member types
// Member `topic_message_timestamps`
#include "ros2_benchmark_interfaces/msg/topic_message_timestamp_array.h"
// Member `topic_message_timestamps`
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_benchmark_interfaces__srv__StartRecording_Request__init(message_memory);
}

void ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_fini_function(void * message_memory)
{
  ros2_benchmark_interfaces__srv__StartRecording_Request__fini(message_memory);
}

size_t ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__size_function__StartRecording_Request__topic_message_timestamps(
  const void * untyped_member)
{
  const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence * member =
    (const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__get_const_function__StartRecording_Request__topic_message_timestamps(
  const void * untyped_member, size_t index)
{
  const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence * member =
    (const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__get_function__StartRecording_Request__topic_message_timestamps(
  void * untyped_member, size_t index)
{
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence * member =
    (ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__fetch_function__StartRecording_Request__topic_message_timestamps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray * item =
    ((const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray *)
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__get_const_function__StartRecording_Request__topic_message_timestamps(untyped_member, index));
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray * value =
    (ros2_benchmark_interfaces__msg__TopicMessageTimestampArray *)(untyped_value);
  *value = *item;
}

void ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__assign_function__StartRecording_Request__topic_message_timestamps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray * item =
    ((ros2_benchmark_interfaces__msg__TopicMessageTimestampArray *)
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__get_function__StartRecording_Request__topic_message_timestamps(untyped_member, index));
  const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray * value =
    (const ros2_benchmark_interfaces__msg__TopicMessageTimestampArray *)(untyped_value);
  *item = *value;
}

bool ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__resize_function__StartRecording_Request__topic_message_timestamps(
  void * untyped_member, size_t size)
{
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence * member =
    (ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence *)(untyped_member);
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__fini(member);
  return ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_member_array[4] = {
  {
    "buffer_length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Request, buffer_length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Request, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "topic_message_timestamps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Request, topic_message_timestamps),  // bytes offset in struct
    NULL,  // default value
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__size_function__StartRecording_Request__topic_message_timestamps,  // size() function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__get_const_function__StartRecording_Request__topic_message_timestamps,  // get_const(index) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__get_function__StartRecording_Request__topic_message_timestamps,  // get(index) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__fetch_function__StartRecording_Request__topic_message_timestamps,  // fetch(index, &value) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__assign_function__StartRecording_Request__topic_message_timestamps,  // assign(index, value) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__resize_function__StartRecording_Request__topic_message_timestamps  // resize(index) function pointer
  },
  {
    "record_data_timeline",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Request, record_data_timeline),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_members = {
  "ros2_benchmark_interfaces__srv",  // message namespace
  "StartRecording_Request",  // message name
  4,  // number of fields
  sizeof(ros2_benchmark_interfaces__srv__StartRecording_Request),
  ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_member_array,  // message members
  ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_type_support_handle = {
  0,
  &ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_benchmark_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording_Request)() {
  ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, msg, TopicMessageTimestampArray)();
  if (!ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_type_support_handle.typesupport_identifier) {
    ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_benchmark_interfaces__srv__StartRecording_Request__rosidl_typesupport_introspection_c__StartRecording_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_recording__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros2_benchmark_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_recording__functions.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_recording__struct.h"


// Include directives for member types
// Member `recorded_topic_message_counts`
#include "ros2_benchmark_interfaces/msg/topic_message_count.h"
// Member `recorded_topic_message_counts`
#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros2_benchmark_interfaces__srv__StartRecording_Response__init(message_memory);
}

void ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_fini_function(void * message_memory)
{
  ros2_benchmark_interfaces__srv__StartRecording_Response__fini(message_memory);
}

size_t ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__size_function__StartRecording_Response__recorded_topic_message_counts(
  const void * untyped_member)
{
  const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * member =
    (const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *)(untyped_member);
  return member->size;
}

const void * ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__get_const_function__StartRecording_Response__recorded_topic_message_counts(
  const void * untyped_member, size_t index)
{
  const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * member =
    (const ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__get_function__StartRecording_Response__recorded_topic_message_counts(
  void * untyped_member, size_t index)
{
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * member =
    (ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__fetch_function__StartRecording_Response__recorded_topic_message_counts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros2_benchmark_interfaces__msg__TopicMessageCount * item =
    ((const ros2_benchmark_interfaces__msg__TopicMessageCount *)
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__get_const_function__StartRecording_Response__recorded_topic_message_counts(untyped_member, index));
  ros2_benchmark_interfaces__msg__TopicMessageCount * value =
    (ros2_benchmark_interfaces__msg__TopicMessageCount *)(untyped_value);
  *value = *item;
}

void ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__assign_function__StartRecording_Response__recorded_topic_message_counts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros2_benchmark_interfaces__msg__TopicMessageCount * item =
    ((ros2_benchmark_interfaces__msg__TopicMessageCount *)
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__get_function__StartRecording_Response__recorded_topic_message_counts(untyped_member, index));
  const ros2_benchmark_interfaces__msg__TopicMessageCount * value =
    (const ros2_benchmark_interfaces__msg__TopicMessageCount *)(untyped_value);
  *item = *value;
}

bool ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__resize_function__StartRecording_Response__recorded_topic_message_counts(
  void * untyped_member, size_t size)
{
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence * member =
    (ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence *)(untyped_member);
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__fini(member);
  return ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recorded_message_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Response, recorded_message_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recorded_topic_message_counts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros2_benchmark_interfaces__srv__StartRecording_Response, recorded_topic_message_counts),  // bytes offset in struct
    NULL,  // default value
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__size_function__StartRecording_Response__recorded_topic_message_counts,  // size() function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__get_const_function__StartRecording_Response__recorded_topic_message_counts,  // get_const(index) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__get_function__StartRecording_Response__recorded_topic_message_counts,  // get(index) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__fetch_function__StartRecording_Response__recorded_topic_message_counts,  // fetch(index, &value) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__assign_function__StartRecording_Response__recorded_topic_message_counts,  // assign(index, value) function pointer
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__resize_function__StartRecording_Response__recorded_topic_message_counts  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_members = {
  "ros2_benchmark_interfaces__srv",  // message namespace
  "StartRecording_Response",  // message name
  3,  // number of fields
  sizeof(ros2_benchmark_interfaces__srv__StartRecording_Response),
  ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_member_array,  // message members
  ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_type_support_handle = {
  0,
  &ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_benchmark_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording_Response)() {
  ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, msg, TopicMessageCount)();
  if (!ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_type_support_handle.typesupport_identifier) {
    ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros2_benchmark_interfaces__srv__StartRecording_Response__rosidl_typesupport_introspection_c__StartRecording_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros2_benchmark_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_recording__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_members = {
  "ros2_benchmark_interfaces__srv",  // service namespace
  "StartRecording",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_Request_message_type_support_handle,
  NULL  // response message
  // ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_Response_message_type_support_handle
};

static rosidl_service_type_support_t ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_type_support_handle = {
  0,
  &ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros2_benchmark_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording)() {
  if (!ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_type_support_handle.typesupport_identifier) {
    ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros2_benchmark_interfaces, srv, StartRecording_Response)()->data;
  }

  return &ros2_benchmark_interfaces__srv__detail__start_recording__rosidl_typesupport_introspection_c__StartRecording_service_type_support_handle;
}
