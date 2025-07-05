// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros2_benchmark_interfaces:srv/StartLoading.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ros2_benchmark_interfaces/srv/detail/start_loading__struct.h"
#include "ros2_benchmark_interfaces/srv/detail/start_loading__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros2_benchmark_interfaces__srv__start_loading__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ros2_benchmark_interfaces.srv._start_loading.StartLoading_Request", full_classname_dest, 65) == 0);
  }
  ros2_benchmark_interfaces__srv__StartLoading_Request * ros_message = _ros_message;
  {  // start_time_offset_ns
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_time_offset_ns");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->start_time_offset_ns = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // end_time_offset_ns
    PyObject * field = PyObject_GetAttrString(_pymsg, "end_time_offset_ns");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->end_time_offset_ns = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // repeat_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "repeat_data");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->repeat_data = (Py_True == field);
    Py_DECREF(field);
  }
  {  // publish_in_real_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "publish_in_real_time");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->publish_in_real_time = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_benchmark_interfaces__srv__start_loading__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartLoading_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_benchmark_interfaces.srv._start_loading");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartLoading_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2_benchmark_interfaces__srv__StartLoading_Request * ros_message = (ros2_benchmark_interfaces__srv__StartLoading_Request *)raw_ros_message;
  {  // start_time_offset_ns
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->start_time_offset_ns);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_time_offset_ns", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // end_time_offset_ns
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->end_time_offset_ns);
    {
      int rc = PyObject_SetAttrString(_pymessage, "end_time_offset_ns", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // repeat_data
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->repeat_data ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "repeat_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // publish_in_real_time
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->publish_in_real_time ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "publish_in_real_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_loading__struct.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_loading__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ros2_benchmark_interfaces/msg/detail/topic_message_timestamp_array__functions.h"
// end nested array functions include
bool ros2_benchmark_interfaces__msg__topic_message_timestamp_array__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2_benchmark_interfaces__msg__topic_message_timestamp_array__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros2_benchmark_interfaces__srv__start_loading__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ros2_benchmark_interfaces.srv._start_loading.StartLoading_Response", full_classname_dest, 66) == 0);
  }
  ros2_benchmark_interfaces__srv__StartLoading_Response * ros_message = _ros_message;
  {  // topic_message_timestamps
    PyObject * field = PyObject_GetAttrString(_pymsg, "topic_message_timestamps");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'topic_message_timestamps'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence__init(&(ros_message->topic_message_timestamps), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ros2_benchmark_interfaces__msg__TopicMessageTimestampArray__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ros2_benchmark_interfaces__msg__TopicMessageTimestampArray * dest = ros_message->topic_message_timestamps.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ros2_benchmark_interfaces__msg__topic_message_timestamp_array__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }
  {  // played_message_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "played_message_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->played_message_count = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_benchmark_interfaces__srv__start_loading__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartLoading_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_benchmark_interfaces.srv._start_loading");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartLoading_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2_benchmark_interfaces__srv__StartLoading_Response * ros_message = (ros2_benchmark_interfaces__srv__StartLoading_Response *)raw_ros_message;
  {  // topic_message_timestamps
    PyObject * field = NULL;
    size_t size = ros_message->topic_message_timestamps.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ros2_benchmark_interfaces__msg__TopicMessageTimestampArray * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->topic_message_timestamps.data[i]);
      PyObject * pyitem = ros2_benchmark_interfaces__msg__topic_message_timestamp_array__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "topic_message_timestamps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // played_message_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->played_message_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "played_message_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
