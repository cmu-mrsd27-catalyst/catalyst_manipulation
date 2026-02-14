// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xarm_msgs:srv/FtAdmittanceParams.idl
// generated code does not contain a copyright notice

#ifndef XARM_MSGS__SRV__DETAIL__FT_ADMITTANCE_PARAMS__STRUCT_H_
#define XARM_MSGS__SRV__DETAIL__FT_ADMITTANCE_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'c_axis'
// Member 'm'
// Member 'k'
// Member 'b'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/FtAdmittanceParams in the package xarm_msgs.
typedef struct xarm_msgs__srv__FtAdmittanceParams_Request
{
  /// task frame (0: base frame. 1: tool frame)
  int16_t coord;
  /// a 6d vector of 0s and 1s. 1 means that robot will be admittance in the corresponding axis of the task frame.
  rosidl_runtime_c__int16__Sequence c_axis;
  /// 6d vector, mass. (kg)
  rosidl_runtime_c__float__Sequence m;
  /// 6d vector, stiffness coefficient
  rosidl_runtime_c__float__Sequence k;
  /// 6d vector, damping coefficient
  rosidl_runtime_c__float__Sequence b;
} xarm_msgs__srv__FtAdmittanceParams_Request;

// Struct for a sequence of xarm_msgs__srv__FtAdmittanceParams_Request.
typedef struct xarm_msgs__srv__FtAdmittanceParams_Request__Sequence
{
  xarm_msgs__srv__FtAdmittanceParams_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__srv__FtAdmittanceParams_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/FtAdmittanceParams in the package xarm_msgs.
typedef struct xarm_msgs__srv__FtAdmittanceParams_Response
{
  int16_t ret;
  rosidl_runtime_c__String message;
} xarm_msgs__srv__FtAdmittanceParams_Response;

// Struct for a sequence of xarm_msgs__srv__FtAdmittanceParams_Response.
typedef struct xarm_msgs__srv__FtAdmittanceParams_Response__Sequence
{
  xarm_msgs__srv__FtAdmittanceParams_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xarm_msgs__srv__FtAdmittanceParams_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XARM_MSGS__SRV__DETAIL__FT_ADMITTANCE_PARAMS__STRUCT_H_
