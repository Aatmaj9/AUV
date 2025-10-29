// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ACTUATOR__STRUCT_H_
#define INTERFACES__MSG__DETAIL__ACTUATOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'actuator_values'
// Member 'covariance'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'actuator_names'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Actuator in the package interfaces.
typedef struct interfaces__msg__Actuator
{
  std_msgs__msg__Header header;
  /// Array of actuator values (rudder, propeller, etc.)
  rosidl_runtime_c__double__Sequence actuator_values;
  /// Array of actuator names for identification
  rosidl_runtime_c__String__Sequence actuator_names;
  /// Covariance matrix in row-major order
  rosidl_runtime_c__double__Sequence covariance;
} interfaces__msg__Actuator;

// Struct for a sequence of interfaces__msg__Actuator.
typedef struct interfaces__msg__Actuator__Sequence
{
  interfaces__msg__Actuator * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Actuator__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__ACTUATOR__STRUCT_H_
