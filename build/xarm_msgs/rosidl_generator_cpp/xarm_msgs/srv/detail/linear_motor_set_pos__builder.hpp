// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xarm_msgs:srv/LinearMotorSetPos.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__LINEAR_MOTOR_SET_POS__BUILDER_HPP_
#define XARM_MSGS__SRV__DETAIL__LINEAR_MOTOR_SET_POS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xarm_msgs/srv/detail/linear_motor_set_pos__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_LinearMotorSetPos_Request_auto_enable
{
public:
  explicit Init_LinearMotorSetPos_Request_auto_enable(::xarm_msgs::srv::LinearMotorSetPos_Request & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::LinearMotorSetPos_Request auto_enable(::xarm_msgs::srv::LinearMotorSetPos_Request::_auto_enable_type arg)
  {
    msg_.auto_enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Request msg_;
};

class Init_LinearMotorSetPos_Request_timeout
{
public:
  explicit Init_LinearMotorSetPos_Request_timeout(::xarm_msgs::srv::LinearMotorSetPos_Request & msg)
  : msg_(msg)
  {}
  Init_LinearMotorSetPos_Request_auto_enable timeout(::xarm_msgs::srv::LinearMotorSetPos_Request::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_LinearMotorSetPos_Request_auto_enable(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Request msg_;
};

class Init_LinearMotorSetPos_Request_wait
{
public:
  explicit Init_LinearMotorSetPos_Request_wait(::xarm_msgs::srv::LinearMotorSetPos_Request & msg)
  : msg_(msg)
  {}
  Init_LinearMotorSetPos_Request_timeout wait(::xarm_msgs::srv::LinearMotorSetPos_Request::_wait_type arg)
  {
    msg_.wait = std::move(arg);
    return Init_LinearMotorSetPos_Request_timeout(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Request msg_;
};

class Init_LinearMotorSetPos_Request_speed
{
public:
  explicit Init_LinearMotorSetPos_Request_speed(::xarm_msgs::srv::LinearMotorSetPos_Request & msg)
  : msg_(msg)
  {}
  Init_LinearMotorSetPos_Request_wait speed(::xarm_msgs::srv::LinearMotorSetPos_Request::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_LinearMotorSetPos_Request_wait(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Request msg_;
};

class Init_LinearMotorSetPos_Request_pos
{
public:
  Init_LinearMotorSetPos_Request_pos()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearMotorSetPos_Request_speed pos(::xarm_msgs::srv::LinearMotorSetPos_Request::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_LinearMotorSetPos_Request_speed(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::LinearMotorSetPos_Request>()
{
  return xarm_msgs::srv::builder::Init_LinearMotorSetPos_Request_pos();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_LinearMotorSetPos_Response_message
{
public:
  explicit Init_LinearMotorSetPos_Response_message(::xarm_msgs::srv::LinearMotorSetPos_Response & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::LinearMotorSetPos_Response message(::xarm_msgs::srv::LinearMotorSetPos_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Response msg_;
};

class Init_LinearMotorSetPos_Response_ret
{
public:
  Init_LinearMotorSetPos_Response_ret()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearMotorSetPos_Response_message ret(::xarm_msgs::srv::LinearMotorSetPos_Response::_ret_type arg)
  {
    msg_.ret = std::move(arg);
    return Init_LinearMotorSetPos_Response_message(msg_);
  }

private:
  ::xarm_msgs::srv::LinearMotorSetPos_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::LinearMotorSetPos_Response>()
{
  return xarm_msgs::srv::builder::Init_LinearMotorSetPos_Response_ret();
}

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__LINEAR_MOTOR_SET_POS__BUILDER_HPP_
