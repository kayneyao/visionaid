// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros2_benchmark_interfaces:srv/StartRecording.idl
// generated code does not contain a copyright notice
#include "ros2_benchmark_interfaces/srv/detail/start_recording__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `topic_message_timestamps`
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__functions.h"

bool
ros2_benchmark_interfaces__srv__StartRecording_Request__init(ros2_benchmark_interfaces__srv__StartRecording_Request * msg)
{
  if (!msg) {
    return false;
  }
  // buffer_length
  // timeout
  // topic_message_timestamps
  if (!ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__init(&msg->topic_message_timestamps, 0)) {
    ros2_benchmark_interfaces__srv__StartRecording_Request__fini(msg);
    return false;
  }
  // record_data_timeline
  msg->record_data_timeline = false;
  return true;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Request__fini(ros2_benchmark_interfaces__srv__StartRecording_Request * msg)
{
  if (!msg) {
    return;
  }
  // buffer_length
  // timeout
  // topic_message_timestamps
  ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__fini(&msg->topic_message_timestamps);
  // record_data_timeline
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Request__are_equal(const ros2_benchmark_interfaces__srv__StartRecording_Request * lhs, const ros2_benchmark_interfaces__srv__StartRecording_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // buffer_length
  if (lhs->buffer_length != rhs->buffer_length) {
    return false;
  }
  // timeout
  if (lhs->timeout != rhs->timeout) {
    return false;
  }
  // topic_message_timestamps
  if (!ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__are_equal(
      &(lhs->topic_message_timestamps), &(rhs->topic_message_timestamps)))
  {
    return false;
  }
  // record_data_timeline
  if (lhs->record_data_timeline != rhs->record_data_timeline) {
    return false;
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Request__copy(
  const ros2_benchmark_interfaces__srv__StartRecording_Request * input,
  ros2_benchmark_interfaces__srv__StartRecording_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // buffer_length
  output->buffer_length = input->buffer_length;
  // timeout
  output->timeout = input->timeout;
  // topic_message_timestamps
  if (!ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__copy(
      &(input->topic_message_timestamps), &(output->topic_message_timestamps)))
  {
    return false;
  }
  // record_data_timeline
  output->record_data_timeline = input->record_data_timeline;
  return true;
}

ros2_benchmark_interfaces__srv__StartRecording_Request *
ros2_benchmark_interfaces__srv__StartRecording_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__StartRecording_Request * msg = (ros2_benchmark_interfaces__srv__StartRecording_Request *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__StartRecording_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_benchmark_interfaces__srv__StartRecording_Request));
  bool success = ros2_benchmark_interfaces__srv__StartRecording_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Request__destroy(ros2_benchmark_interfaces__srv__StartRecording_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_benchmark_interfaces__srv__StartRecording_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__init(ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__StartRecording_Request * data = NULL;

  if (size) {
    data = (ros2_benchmark_interfaces__srv__StartRecording_Request *)allocator.zero_allocate(size, sizeof(ros2_benchmark_interfaces__srv__StartRecording_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_benchmark_interfaces__srv__StartRecording_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_benchmark_interfaces__srv__StartRecording_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__fini(ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_benchmark_interfaces__srv__StartRecording_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence *
ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * array = (ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__destroy(ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__are_equal(const ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * lhs, const ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__StartRecording_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence__copy(
  const ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * input,
  ros2_benchmark_interfaces__srv__StartRecording_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_benchmark_interfaces__srv__StartRecording_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_benchmark_interfaces__srv__StartRecording_Request * data =
      (ros2_benchmark_interfaces__srv__StartRecording_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_benchmark_interfaces__srv__StartRecording_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_benchmark_interfaces__srv__StartRecording_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__StartRecording_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `recorded_topic_message_counts`
#include "ros2_benchmark_interfaces/msg/detail/topic_message_count__functions.h"

bool
ros2_benchmark_interfaces__srv__StartRecording_Response__init(ros2_benchmark_interfaces__srv__StartRecording_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  msg->success = false;
  // recorded_message_count
  // recorded_topic_message_counts
  if (!ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__init(&msg->recorded_topic_message_counts, 0)) {
    ros2_benchmark_interfaces__srv__StartRecording_Response__fini(msg);
    return false;
  }
  return true;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Response__fini(ros2_benchmark_interfaces__srv__StartRecording_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // recorded_message_count
  // recorded_topic_message_counts
  ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__fini(&msg->recorded_topic_message_counts);
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Response__are_equal(const ros2_benchmark_interfaces__srv__StartRecording_Response * lhs, const ros2_benchmark_interfaces__srv__StartRecording_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // recorded_message_count
  if (lhs->recorded_message_count != rhs->recorded_message_count) {
    return false;
  }
  // recorded_topic_message_counts
  if (!ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__are_equal(
      &(lhs->recorded_topic_message_counts), &(rhs->recorded_topic_message_counts)))
  {
    return false;
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Response__copy(
  const ros2_benchmark_interfaces__srv__StartRecording_Response * input,
  ros2_benchmark_interfaces__srv__StartRecording_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // recorded_message_count
  output->recorded_message_count = input->recorded_message_count;
  // recorded_topic_message_counts
  if (!ros2_benchmark_interfaces__msg__TopicMessageCount__Sequence__copy(
      &(input->recorded_topic_message_counts), &(output->recorded_topic_message_counts)))
  {
    return false;
  }
  return true;
}

ros2_benchmark_interfaces__srv__StartRecording_Response *
ros2_benchmark_interfaces__srv__StartRecording_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__StartRecording_Response * msg = (ros2_benchmark_interfaces__srv__StartRecording_Response *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__StartRecording_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros2_benchmark_interfaces__srv__StartRecording_Response));
  bool success = ros2_benchmark_interfaces__srv__StartRecording_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Response__destroy(ros2_benchmark_interfaces__srv__StartRecording_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros2_benchmark_interfaces__srv__StartRecording_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__init(ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__StartRecording_Response * data = NULL;

  if (size) {
    data = (ros2_benchmark_interfaces__srv__StartRecording_Response *)allocator.zero_allocate(size, sizeof(ros2_benchmark_interfaces__srv__StartRecording_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros2_benchmark_interfaces__srv__StartRecording_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros2_benchmark_interfaces__srv__StartRecording_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__fini(ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ros2_benchmark_interfaces__srv__StartRecording_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence *
ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * array = (ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence *)allocator.allocate(sizeof(ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__destroy(ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__are_equal(const ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * lhs, const ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__StartRecording_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence__copy(
  const ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * input,
  ros2_benchmark_interfaces__srv__StartRecording_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros2_benchmark_interfaces__srv__StartRecording_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros2_benchmark_interfaces__srv__StartRecording_Response * data =
      (ros2_benchmark_interfaces__srv__StartRecording_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros2_benchmark_interfaces__srv__StartRecording_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros2_benchmark_interfaces__srv__StartRecording_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros2_benchmark_interfaces__srv__StartRecording_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
