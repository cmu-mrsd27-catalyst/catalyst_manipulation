// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from catalyst_interfaces:srv/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__STRUCT_H_
#define CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GripperCommand in the package catalyst_interfaces.
typedef struct catalyst_interfaces__srv__GripperCommand_Request
{
  rosidl_runtime_c__String command;
} catalyst_interfaces__srv__GripperCommand_Request;

// Struct for a sequence of catalyst_interfaces__srv__GripperCommand_Request.
typedef struct catalyst_interfaces__srv__GripperCommand_Request__Sequence
{
  catalyst_interfaces__srv__GripperCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catalyst_interfaces__srv__GripperCommand_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GripperCommand in the package catalyst_interfaces.
typedef struct catalyst_interfaces__srv__GripperCommand_Response
{
  rosidl_runtime_c__String response;
} catalyst_interfaces__srv__GripperCommand_Response;

// Struct for a sequence of catalyst_interfaces__srv__GripperCommand_Response.
typedef struct catalyst_interfaces__srv__GripperCommand_Response__Sequence
{
  catalyst_interfaces__srv__GripperCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} catalyst_interfaces__srv__GripperCommand_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__STRUCT_H_
