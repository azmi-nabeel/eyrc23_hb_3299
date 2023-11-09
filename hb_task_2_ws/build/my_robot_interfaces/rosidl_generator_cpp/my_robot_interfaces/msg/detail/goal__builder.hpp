// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/Goal.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__GOAL__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__GOAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/goal__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_Goal_theta
{
public:
  explicit Init_Goal_theta(::my_robot_interfaces::msg::Goal & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::Goal theta(::my_robot_interfaces::msg::Goal::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::Goal msg_;
};

class Init_Goal_y
{
public:
  explicit Init_Goal_y(::my_robot_interfaces::msg::Goal & msg)
  : msg_(msg)
  {}
  Init_Goal_theta y(::my_robot_interfaces::msg::Goal::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Goal_theta(msg_);
  }

private:
  ::my_robot_interfaces::msg::Goal msg_;
};

class Init_Goal_x
{
public:
  explicit Init_Goal_x(::my_robot_interfaces::msg::Goal & msg)
  : msg_(msg)
  {}
  Init_Goal_y x(::my_robot_interfaces::msg::Goal::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Goal_y(msg_);
  }

private:
  ::my_robot_interfaces::msg::Goal msg_;
};

class Init_Goal_bot_id
{
public:
  Init_Goal_bot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Goal_x bot_id(::my_robot_interfaces::msg::Goal::_bot_id_type arg)
  {
    msg_.bot_id = std::move(arg);
    return Init_Goal_x(msg_);
  }

private:
  ::my_robot_interfaces::msg::Goal msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::Goal>()
{
  return my_robot_interfaces::msg::builder::Init_Goal_bot_id();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__GOAL__BUILDER_HPP_
