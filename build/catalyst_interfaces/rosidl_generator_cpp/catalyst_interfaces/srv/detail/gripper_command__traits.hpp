// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from catalyst_interfaces:srv/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__TRAITS_HPP_
#define CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "catalyst_interfaces/srv/detail/gripper_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace catalyst_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperCommand_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperCommand_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace catalyst_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use catalyst_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const catalyst_interfaces::srv::GripperCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  catalyst_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use catalyst_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const catalyst_interfaces::srv::GripperCommand_Request & msg)
{
  return catalyst_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<catalyst_interfaces::srv::GripperCommand_Request>()
{
  return "catalyst_interfaces::srv::GripperCommand_Request";
}

template<>
inline const char * name<catalyst_interfaces::srv::GripperCommand_Request>()
{
  return "catalyst_interfaces/srv/GripperCommand_Request";
}

template<>
struct has_fixed_size<catalyst_interfaces::srv::GripperCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<catalyst_interfaces::srv::GripperCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<catalyst_interfaces::srv::GripperCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace catalyst_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GripperCommand_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperCommand_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace catalyst_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use catalyst_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const catalyst_interfaces::srv::GripperCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  catalyst_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use catalyst_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const catalyst_interfaces::srv::GripperCommand_Response & msg)
{
  return catalyst_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<catalyst_interfaces::srv::GripperCommand_Response>()
{
  return "catalyst_interfaces::srv::GripperCommand_Response";
}

template<>
inline const char * name<catalyst_interfaces::srv::GripperCommand_Response>()
{
  return "catalyst_interfaces/srv/GripperCommand_Response";
}

template<>
struct has_fixed_size<catalyst_interfaces::srv::GripperCommand_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<catalyst_interfaces::srv::GripperCommand_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<catalyst_interfaces::srv::GripperCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<catalyst_interfaces::srv::GripperCommand>()
{
  return "catalyst_interfaces::srv::GripperCommand";
}

template<>
inline const char * name<catalyst_interfaces::srv::GripperCommand>()
{
  return "catalyst_interfaces/srv/GripperCommand";
}

template<>
struct has_fixed_size<catalyst_interfaces::srv::GripperCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<catalyst_interfaces::srv::GripperCommand_Request>::value &&
    has_fixed_size<catalyst_interfaces::srv::GripperCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<catalyst_interfaces::srv::GripperCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<catalyst_interfaces::srv::GripperCommand_Request>::value &&
    has_bounded_size<catalyst_interfaces::srv::GripperCommand_Response>::value
  >
{
};

template<>
struct is_service<catalyst_interfaces::srv::GripperCommand>
  : std::true_type
{
};

template<>
struct is_service_request<catalyst_interfaces::srv::GripperCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<catalyst_interfaces::srv::GripperCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__TRAITS_HPP_
