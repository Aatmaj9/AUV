// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/actuator__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `actuator_values`
// Member `covariance`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `actuator_names`
#include "rosidl_runtime_c/string_functions.h"

bool
interfaces__msg__Actuator__init(interfaces__msg__Actuator * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    interfaces__msg__Actuator__fini(msg);
    return false;
  }
  // actuator_values
  if (!rosidl_runtime_c__double__Sequence__init(&msg->actuator_values, 0)) {
    interfaces__msg__Actuator__fini(msg);
    return false;
  }
  // actuator_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->actuator_names, 0)) {
    interfaces__msg__Actuator__fini(msg);
    return false;
  }
  // covariance
  if (!rosidl_runtime_c__double__Sequence__init(&msg->covariance, 0)) {
    interfaces__msg__Actuator__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__Actuator__fini(interfaces__msg__Actuator * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // actuator_values
  rosidl_runtime_c__double__Sequence__fini(&msg->actuator_values);
  // actuator_names
  rosidl_runtime_c__String__Sequence__fini(&msg->actuator_names);
  // covariance
  rosidl_runtime_c__double__Sequence__fini(&msg->covariance);
}

bool
interfaces__msg__Actuator__are_equal(const interfaces__msg__Actuator * lhs, const interfaces__msg__Actuator * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // actuator_values
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->actuator_values), &(rhs->actuator_values)))
  {
    return false;
  }
  // actuator_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->actuator_names), &(rhs->actuator_names)))
  {
    return false;
  }
  // covariance
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->covariance), &(rhs->covariance)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__Actuator__copy(
  const interfaces__msg__Actuator * input,
  interfaces__msg__Actuator * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // actuator_values
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->actuator_values), &(output->actuator_values)))
  {
    return false;
  }
  // actuator_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->actuator_names), &(output->actuator_names)))
  {
    return false;
  }
  // covariance
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->covariance), &(output->covariance)))
  {
    return false;
  }
  return true;
}

interfaces__msg__Actuator *
interfaces__msg__Actuator__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Actuator * msg = (interfaces__msg__Actuator *)allocator.allocate(sizeof(interfaces__msg__Actuator), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__Actuator));
  bool success = interfaces__msg__Actuator__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__Actuator__destroy(interfaces__msg__Actuator * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__Actuator__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__Actuator__Sequence__init(interfaces__msg__Actuator__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Actuator * data = NULL;

  if (size) {
    data = (interfaces__msg__Actuator *)allocator.zero_allocate(size, sizeof(interfaces__msg__Actuator), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__Actuator__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__Actuator__fini(&data[i - 1]);
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
interfaces__msg__Actuator__Sequence__fini(interfaces__msg__Actuator__Sequence * array)
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
      interfaces__msg__Actuator__fini(&array->data[i]);
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

interfaces__msg__Actuator__Sequence *
interfaces__msg__Actuator__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Actuator__Sequence * array = (interfaces__msg__Actuator__Sequence *)allocator.allocate(sizeof(interfaces__msg__Actuator__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__Actuator__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__Actuator__Sequence__destroy(interfaces__msg__Actuator__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__Actuator__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__Actuator__Sequence__are_equal(const interfaces__msg__Actuator__Sequence * lhs, const interfaces__msg__Actuator__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__Actuator__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__Actuator__Sequence__copy(
  const interfaces__msg__Actuator__Sequence * input,
  interfaces__msg__Actuator__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__Actuator);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__Actuator * data =
      (interfaces__msg__Actuator *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__Actuator__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__Actuator__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__Actuator__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
