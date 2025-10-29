// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "interfaces/msg/detail/actuator__rosidl_typesupport_introspection_c.h"
#include "interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "interfaces/msg/detail/actuator__functions.h"
#include "interfaces/msg/detail/actuator__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `actuator_values`
// Member `covariance`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `actuator_names`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  interfaces__msg__Actuator__init(message_memory);
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_fini_function(void * message_memory)
{
  interfaces__msg__Actuator__fini(message_memory);
}

size_t interfaces__msg__Actuator__rosidl_typesupport_introspection_c__size_function__Actuator__actuator_values(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__actuator_values(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__actuator_values(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__fetch_function__Actuator__actuator_values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__actuator_values(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__assign_function__Actuator__actuator_values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__actuator_values(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interfaces__msg__Actuator__rosidl_typesupport_introspection_c__resize_function__Actuator__actuator_values(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t interfaces__msg__Actuator__rosidl_typesupport_introspection_c__size_function__Actuator__actuator_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__actuator_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__actuator_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__fetch_function__Actuator__actuator_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__actuator_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__assign_function__Actuator__actuator_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__actuator_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool interfaces__msg__Actuator__rosidl_typesupport_introspection_c__resize_function__Actuator__actuator_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t interfaces__msg__Actuator__rosidl_typesupport_introspection_c__size_function__Actuator__covariance(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__covariance(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__covariance(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__fetch_function__Actuator__covariance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__covariance(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void interfaces__msg__Actuator__rosidl_typesupport_introspection_c__assign_function__Actuator__covariance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__covariance(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool interfaces__msg__Actuator__rosidl_typesupport_introspection_c__resize_function__Actuator__covariance(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Actuator, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actuator_values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Actuator, actuator_values),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__size_function__Actuator__actuator_values,  // size() function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__actuator_values,  // get_const(index) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__actuator_values,  // get(index) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__fetch_function__Actuator__actuator_values,  // fetch(index, &value) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__assign_function__Actuator__actuator_values,  // assign(index, value) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__resize_function__Actuator__actuator_values  // resize(index) function pointer
  },
  {
    "actuator_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Actuator, actuator_names),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__size_function__Actuator__actuator_names,  // size() function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__actuator_names,  // get_const(index) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__actuator_names,  // get(index) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__fetch_function__Actuator__actuator_names,  // fetch(index, &value) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__assign_function__Actuator__actuator_names,  // assign(index, value) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__resize_function__Actuator__actuator_names  // resize(index) function pointer
  },
  {
    "covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(interfaces__msg__Actuator, covariance),  // bytes offset in struct
    NULL,  // default value
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__size_function__Actuator__covariance,  // size() function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_const_function__Actuator__covariance,  // get_const(index) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__get_function__Actuator__covariance,  // get(index) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__fetch_function__Actuator__covariance,  // fetch(index, &value) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__assign_function__Actuator__covariance,  // assign(index, value) function pointer
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__resize_function__Actuator__covariance  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_members = {
  "interfaces__msg",  // message namespace
  "Actuator",  // message name
  4,  // number of fields
  sizeof(interfaces__msg__Actuator),
  interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_member_array,  // message members
  interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_init_function,  // function to initialize message memory (memory has to be allocated)
  interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_type_support_handle = {
  0,
  &interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, interfaces, msg, Actuator)() {
  interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_type_support_handle.typesupport_identifier) {
    interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &interfaces__msg__Actuator__rosidl_typesupport_introspection_c__Actuator_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
