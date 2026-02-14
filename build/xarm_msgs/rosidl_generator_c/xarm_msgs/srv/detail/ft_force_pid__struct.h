// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xarm_msgs:srv/FtForcePid.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__FT_FORCE_PID__STRUCT_H_
#define XARM_MSGS__SRV__DETAIL__FT_FORCE_PID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'kp'
// Member 'ki'
// Member 'kd'
// Member 'xe_limit'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/FtForcePid in the package xarm_msgs.
typedef struct xarm_msgs__srv__FtForcePid_Request
{
  /// 6d vector, proportional gain.
  rosidl_runtime_c__float__Sequence kp;
  /// 6d vector, integral gain.
  rosidl_runtime_c__float__Sequence ki;
  /// 6d vector, differential gain.
  rosidl_runtime_c__float__Sequence kd;
  /// 6d vector. for compliant axes, these values are the maximum allowed tcp speed along/about the axis. mm/s
  rosidl_runtime_c__float__Sequence xe_limit;
} xarm_msgs__srv__FtForcePid_Request;

// Struct for a sequence of xarm_msgs__srv__FtForcePid_Request.
typedef struct xarm_msgs__srv__FtForcePid_Request__Sequence
{
  xarm_msgs__srv__FtForcePid_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__srv__FtForcePid_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FtForcePid in the package xarm_msgs.
typedef struct xarm_msgs__srv__FtForcePid_Response
{
  int16_t ret;
  rosidl_runtime_c__String message;
} xarm_msgs__srv__FtForcePid_Response;

// Struct for a sequence of xarm_msgs__srv__FtForcePid_Response.
typedef struct xarm_msgs__srv__FtForcePid_Response__Sequence
{
  xarm_msgs__srv__FtForcePid_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__srv__FtForcePid_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__SRV__DETAIL__FT_FORCE_PID__STRUCT_H_
