// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros2_benchmark_interfaces:srv/StartMonitoring.idl
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
#include "ros2_benchmark_interfaces/srv/detail/start_monitoring__struct.h"
#include "ros2_benchmark_interfaces/srv/detail/start_monitoring__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros2_benchmark_interfaces__srv__start_monitoring__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[72];
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
    assert(strncmp("ros2_benchmark_interfaces.srv._start_monitoring.StartMonitoring_Request", full_classname_dest, 71) == 0);
  }
  ros2_benchmark_interfaces__srv__StartMonitoring_Request * ros_message = _ros_message;
  {  // message_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "message_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->message_count = PyLong_AsUnsignedLongLong(field);
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
  {  // record_start_timestamps
    PyObject * field = PyObject_GetAttrString(_pymsg, "record_start_timestamps");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->record_start_timestamps = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_benchmark_interfaces__srv__start_monitoring__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartMonitoring_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_benchmark_interfaces.srv._start_monitoring");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartMonitoring_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros2_benchmark_interfaces__srv__StartMonitoring_Request * ros_message = (ros2_benchmark_interfaces__srv__StartMonitoring_Request *)raw_ros_message;
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
  {  // record_start_timestamps
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->record_start_timestamps ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "record_start_timestamps", field);
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
// #include "ros2_benchmark_interfaces/srv/detail/start_monitoring__struct.h"
// already included above
// #include "ros2_benchmark_interfaces/srv/detail/start_monitoring__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros2_benchmark_interfaces__srv__start_monitoring__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[73];
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
    assert(strncmp("ros2_benchmark_interfaces.srv._start_monitoring.StartMonitoring_Response", full_classname_dest, 72) == 0);
  }
  ros2_benchmark_interfaces__srv__StartMonitoring_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros2_benchmark_interfaces__srv__start_monitoring__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartMonitoring_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros2_benchmark_interfaces.srv._start_monitoring");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartMonitoring_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
