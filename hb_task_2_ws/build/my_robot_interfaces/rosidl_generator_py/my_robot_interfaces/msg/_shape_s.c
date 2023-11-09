// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from my_robot_interfaces:msg/Shape.idl
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
#include "my_robot_interfaces/msg/detail/shape__struct.h"
#include "my_robot_interfaces/msg/detail/shape__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool my_robot_interfaces__msg__shape__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
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
    assert(strncmp("my_robot_interfaces.msg._shape.Shape", full_classname_dest, 36) == 0);
  }
  my_robot_interfaces__msg__Shape * ros_message = _ros_message;
  {  // shape_dimension
    PyObject * field = PyObject_GetAttrString(_pymsg, "shape_dimension");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shape_dimension = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // shape_theta
    PyObject * field = PyObject_GetAttrString(_pymsg, "shape_theta");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shape_theta = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * my_robot_interfaces__msg__shape__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Shape */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("my_robot_interfaces.msg._shape");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Shape");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  my_robot_interfaces__msg__Shape * ros_message = (my_robot_interfaces__msg__Shape *)raw_ros_message;
  {  // shape_dimension
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shape_dimension);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shape_dimension", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shape_theta
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shape_theta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shape_theta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
