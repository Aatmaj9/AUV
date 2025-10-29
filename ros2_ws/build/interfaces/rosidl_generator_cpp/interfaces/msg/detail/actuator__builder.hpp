// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ACTUATOR__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ACTUATOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/actuator__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Actuator_covariance
{
public:
  explicit Init_Actuator_covariance(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Actuator covariance(::interfaces::msg::Actuator::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_actuator_names
{
public:
  explicit Init_Actuator_actuator_names(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_covariance actuator_names(::interfaces::msg::Actuator::_actuator_names_type arg)
  {
    msg_.actuator_names = std::move(arg);
    return Init_Actuator_covariance(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_actuator_values
{
public:
  explicit Init_Actuator_actuator_values(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_actuator_names actuator_values(::interfaces::msg::Actuator::_actuator_values_type arg)
  {
    msg_.actuator_values = std::move(arg);
    return Init_Actuator_actuator_names(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_header
{
public:
  Init_Actuator_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Actuator_actuator_values header(::interfaces::msg::Actuator::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Actuator_actuator_values(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Actuator>()
{
  return interfaces::msg::builder::Init_Actuator_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ACTUATOR__BUILDER_HPP_
