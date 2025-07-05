// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros2_benchmark_interfaces:srv/PlayMessages.idl
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
#include "ros2_benchmark_interfaces/srv/detail/play_messages__struct.h"
#include "ros2_benchmark_interfaces/srv/detail/play_messages__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros2_benchmark_interfaces__srv__play_messages__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros2_benchmark_interfaces.srv._play_messages.PlayMessages_Request", full_classname_dest, 65) == 0);
  }
  ros2_benchmark_interfaces__srv__PlayMessages_Request * ros_message = _ros_message;
  {  // playback_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "playback_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->playback_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // message_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "message_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->message_count = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // target_publisher_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_publisher_rate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_publisher_rate = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // enforce_publisher_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "enforce_publisher_rate");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->enforce_publisher_rate = (Py_True == field);
    Py_DECREF(field);
  }
  {  // revise_timestamps_as_message_ids
    PyObject * field = PyObject_GetAttrString(_pymsg, "revise_timestamps_as_message_ids");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->revise_timestamps_as_message_ids = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_benchmark_interfaces__srv__play_messages__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PlayMessages_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_benchmark_interfaces.srv._play_messages");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PlayMessages_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2_benchmark_interfaces__srv__PlayMessages_Request * ros_message = (ros2_benchmark_interfaces__srv__PlayMessages_Request *)raw_ros_message;
  {  // playback_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->playback_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "playback_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->message_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "message_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_publisher_rate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_publisher_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_publisher_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enforce_publisher_rate
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->enforce_publisher_rate ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enforce_publisher_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // revise_timestamps_as_message_ids
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->revise_timestamps_as_message_ids ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "revise_timestamps_as_message_ids", field);
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
// #include "ros2_benchmark_interfaces/srv/detail/play_messages__struct.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/play_messages__functions.h"

bool ros2_benchmark_interfaces__msg__timestamped_message_array__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros2_benchmark_interfaces__msg__timestamped_message_array__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros2_benchmark_interfaces__srv__play_messages__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros2_benchmark_interfaces.srv._play_messages.PlayMessages_Response", full_classname_dest, 66) == 0);
  }
  ros2_benchmark_interfaces__srv__PlayMessages_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // timestamps
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamps");
    if (!field) {
      return false;
    }
    if (!ros2_benchmark_interfaces__msg__timestamped_message_array__convert_from_py(field, &ros_message->timestamps)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_benchmark_interfaces__srv__play_messages__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PlayMessages_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_benchmark_interfaces.srv._play_messages");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PlayMessages_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2_benchmark_interfaces__srv__PlayMessages_Response * ros_message = (ros2_benchmark_interfaces__srv__PlayMessages_Response *)raw_ros_message;
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
  {  // timestamps
    PyObject * field = NULL;
    field = ros2_benchmark_interfaces__msg__timestamped_message_array__convert_to_py(&ros_message->timestamps);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
