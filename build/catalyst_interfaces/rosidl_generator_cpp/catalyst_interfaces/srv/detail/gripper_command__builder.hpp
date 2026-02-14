// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from catalyst_interfaces:srv/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_
#define CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "catalyst_interfaces/srv/detail/gripper_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace catalyst_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperCommand_Request_command
{
public:
  Init_GripperCommand_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::catalyst_interfaces::srv::GripperCommand_Request command(::catalyst_interfaces::srv::GripperCommand_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::catalyst_interfaces::srv::GripperCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::catalyst_interfaces::srv::GripperCommand_Request>()
{
  return catalyst_interfaces::srv::builder::Init_GripperCommand_Request_command();
}

}  // namespace catalyst_interfaces


namespace catalyst_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperCommand_Response_response
{
public:
  Init_GripperCommand_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::catalyst_interfaces::srv::GripperCommand_Response response(::catalyst_interfaces::srv::GripperCommand_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::catalyst_interfaces::srv::GripperCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::catalyst_interfaces::srv::GripperCommand_Response>()
{
  return catalyst_interfaces::srv::builder::Init_GripperCommand_Response_response();
}

}  // namespace catalyst_interfaces

#endif  // CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__BUILDER_HPP_
