// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from xarm_msgs:srv/FtForceParams.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__FT_FORCE_PARAMS__BUILDER_HPP_
#define XARM_MSGS__SRV__DETAIL__FT_FORCE_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "xarm_msgs/srv/detail/ft_force_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_FtForceParams_Request_xe_limit
{
public:
  explicit Init_FtForceParams_Request_xe_limit(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::FtForceParams_Request xe_limit(::xarm_msgs::srv::FtForceParams_Request::_xe_limit_type arg)
  {
    msg_.xe_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_kd
{
public:
  explicit Init_FtForceParams_Request_kd(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  Init_FtForceParams_Request_xe_limit kd(::xarm_msgs::srv::FtForceParams_Request::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_FtForceParams_Request_xe_limit(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_ki
{
public:
  explicit Init_FtForceParams_Request_ki(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  Init_FtForceParams_Request_kd ki(::xarm_msgs::srv::FtForceParams_Request::_ki_type arg)
  {
    msg_.ki = std::move(arg);
    return Init_FtForceParams_Request_kd(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_kp
{
public:
  explicit Init_FtForceParams_Request_kp(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  Init_FtForceParams_Request_ki kp(::xarm_msgs::srv::FtForceParams_Request::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_FtForceParams_Request_ki(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_limits
{
public:
  explicit Init_FtForceParams_Request_limits(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  Init_FtForceParams_Request_kp limits(::xarm_msgs::srv::FtForceParams_Request::_limits_type arg)
  {
    msg_.limits = std::move(arg);
    return Init_FtForceParams_Request_kp(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_ref
{
public:
  explicit Init_FtForceParams_Request_ref(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  Init_FtForceParams_Request_limits ref(::xarm_msgs::srv::FtForceParams_Request::_ref_type arg)
  {
    msg_.ref = std::move(arg);
    return Init_FtForceParams_Request_limits(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_c_axis
{
public:
  explicit Init_FtForceParams_Request_c_axis(::xarm_msgs::srv::FtForceParams_Request & msg)
  : msg_(msg)
  {}
  Init_FtForceParams_Request_ref c_axis(::xarm_msgs::srv::FtForceParams_Request::_c_axis_type arg)
  {
    msg_.c_axis = std::move(arg);
    return Init_FtForceParams_Request_ref(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

class Init_FtForceParams_Request_coord
{
public:
  Init_FtForceParams_Request_coord()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FtForceParams_Request_c_axis coord(::xarm_msgs::srv::FtForceParams_Request::_coord_type arg)
  {
    msg_.coord = std::move(arg);
    return Init_FtForceParams_Request_c_axis(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::FtForceParams_Request>()
{
  return xarm_msgs::srv::builder::Init_FtForceParams_Request_coord();
}

}  // namespace xarm_msgs


namespace xarm_msgs
{

namespace srv
{

namespace builder
{

class Init_FtForceParams_Response_message
{
public:
  explicit Init_FtForceParams_Response_message(::xarm_msgs::srv::FtForceParams_Response & msg)
  : msg_(msg)
  {}
  ::xarm_msgs::srv::FtForceParams_Response message(::xarm_msgs::srv::FtForceParams_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Response msg_;
};

class Init_FtForceParams_Response_ret
{
public:
  Init_FtForceParams_Response_ret()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FtForceParams_Response_message ret(::xarm_msgs::srv::FtForceParams_Response::_ret_type arg)
  {
    msg_.ret = std::move(arg);
    return Init_FtForceParams_Response_message(msg_);
  }

private:
  ::xarm_msgs::srv::FtForceParams_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::xarm_msgs::srv::FtForceParams_Response>()
{
  return xarm_msgs::srv::builder::Init_FtForceParams_Response_ret();
}

}  // namespace xarm_msgs

#endif  // XARM_MSGS__SRV__DETAIL__FT_FORCE_PARAMS__BUILDER_HPP_
