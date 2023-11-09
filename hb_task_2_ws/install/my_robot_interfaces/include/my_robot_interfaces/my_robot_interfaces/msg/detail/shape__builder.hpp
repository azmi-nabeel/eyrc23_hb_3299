// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/Shape.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__SHAPE__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__SHAPE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/shape__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_Shape_shape_theta
{
public:
  explicit Init_Shape_shape_theta(::my_robot_interfaces::msg::Shape & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::Shape shape_theta(::my_robot_interfaces::msg::Shape::_shape_theta_type arg)
  {
    msg_.shape_theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::Shape msg_;
};

class Init_Shape_shape_dimension
{
public:
  Init_Shape_shape_dimension()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Shape_shape_theta shape_dimension(::my_robot_interfaces::msg::Shape::_shape_dimension_type arg)
  {
    msg_.shape_dimension = std::move(arg);
    return Init_Shape_shape_theta(msg_);
  }

private:
  ::my_robot_interfaces::msg::Shape msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::Shape>()
{
  return my_robot_interfaces::msg::builder::Init_Shape_shape_dimension();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__SHAPE__BUILDER_HPP_
