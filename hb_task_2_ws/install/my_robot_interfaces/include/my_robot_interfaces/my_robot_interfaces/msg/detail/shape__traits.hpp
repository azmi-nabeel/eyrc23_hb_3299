// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_robot_interfaces:msg/Shape.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__SHAPE__TRAITS_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__SHAPE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_robot_interfaces/msg/detail/shape__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Shape & msg,
  std::ostream & out)
{
  out << "{";
  // member: shape_dimension
  {
    out << "shape_dimension: ";
    rosidl_generator_traits::value_to_yaml(msg.shape_dimension, out);
    out << ", ";
  }

  // member: shape_theta
  {
    out << "shape_theta: ";
    rosidl_generator_traits::value_to_yaml(msg.shape_theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Shape & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: shape_dimension
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shape_dimension: ";
    rosidl_generator_traits::value_to_yaml(msg.shape_dimension, out);
    out << "\n";
  }

  // member: shape_theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shape_theta: ";
    rosidl_generator_traits::value_to_yaml(msg.shape_theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Shape & msg, bool use_flow_style = false)
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

}  // namespace my_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use my_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_robot_interfaces::msg::Shape & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_robot_interfaces::msg::Shape & msg)
{
  return my_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_robot_interfaces::msg::Shape>()
{
  return "my_robot_interfaces::msg::Shape";
}

template<>
inline const char * name<my_robot_interfaces::msg::Shape>()
{
  return "my_robot_interfaces/msg/Shape";
}

template<>
struct has_fixed_size<my_robot_interfaces::msg::Shape>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_robot_interfaces::msg::Shape>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_robot_interfaces::msg::Shape>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__SHAPE__TRAITS_HPP_
