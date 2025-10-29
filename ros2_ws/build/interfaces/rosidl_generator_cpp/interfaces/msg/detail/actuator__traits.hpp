// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ACTUATOR__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__ACTUATOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/actuator__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Actuator & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: actuator_values
  {
    if (msg.actuator_values.size() == 0) {
      out << "actuator_values: []";
    } else {
      out << "actuator_values: [";
      size_t pending_items = msg.actuator_values.size();
      for (auto item : msg.actuator_values) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: actuator_names
  {
    if (msg.actuator_names.size() == 0) {
      out << "actuator_names: []";
    } else {
      out << "actuator_names: [";
      size_t pending_items = msg.actuator_names.size();
      for (auto item : msg.actuator_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: covariance
  {
    if (msg.covariance.size() == 0) {
      out << "covariance: []";
    } else {
      out << "covariance: [";
      size_t pending_items = msg.covariance.size();
      for (auto item : msg.covariance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Actuator & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: actuator_values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.actuator_values.size() == 0) {
      out << "actuator_values: []\n";
    } else {
      out << "actuator_values:\n";
      for (auto item : msg.actuator_values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: actuator_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.actuator_names.size() == 0) {
      out << "actuator_names: []\n";
    } else {
      out << "actuator_names:\n";
      for (auto item : msg.actuator_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.covariance.size() == 0) {
      out << "covariance: []\n";
    } else {
      out << "covariance:\n";
      for (auto item : msg.covariance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Actuator & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::Actuator & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::Actuator & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::Actuator>()
{
  return "interfaces::msg::Actuator";
}

template<>
inline const char * name<interfaces::msg::Actuator>()
{
  return "interfaces/msg/Actuator";
}

template<>
struct has_fixed_size<interfaces::msg::Actuator>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::Actuator>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::Actuator>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__ACTUATOR__TRAITS_HPP_
