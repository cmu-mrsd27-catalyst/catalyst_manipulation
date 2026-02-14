// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from catalyst_interfaces:srv/GripperCommand.idl
// generated code does not contain a copyright notice

#ifndef CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__FUNCTIONS_H_
#define CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "catalyst_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "catalyst_interfaces/srv/detail/gripper_command__struct.h"

/// Initialize srv/GripperCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * catalyst_interfaces__srv__GripperCommand_Request
 * )) before or use
 * catalyst_interfaces__srv__GripperCommand_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Request__init(catalyst_interfaces__srv__GripperCommand_Request * msg);

/// Finalize srv/GripperCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Request__fini(catalyst_interfaces__srv__GripperCommand_Request * msg);

/// Create srv/GripperCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * catalyst_interfaces__srv__GripperCommand_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
catalyst_interfaces__srv__GripperCommand_Request *
catalyst_interfaces__srv__GripperCommand_Request__create();

/// Destroy srv/GripperCommand message.
/**
 * It calls
 * catalyst_interfaces__srv__GripperCommand_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Request__destroy(catalyst_interfaces__srv__GripperCommand_Request * msg);

/// Check for srv/GripperCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Request__are_equal(const catalyst_interfaces__srv__GripperCommand_Request * lhs, const catalyst_interfaces__srv__GripperCommand_Request * rhs);

/// Copy a srv/GripperCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Request__copy(
  const catalyst_interfaces__srv__GripperCommand_Request * input,
  catalyst_interfaces__srv__GripperCommand_Request * output);

/// Initialize array of srv/GripperCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * catalyst_interfaces__srv__GripperCommand_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Request__Sequence__init(catalyst_interfaces__srv__GripperCommand_Request__Sequence * array, size_t size);

/// Finalize array of srv/GripperCommand messages.
/**
 * It calls
 * catalyst_interfaces__srv__GripperCommand_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Request__Sequence__fini(catalyst_interfaces__srv__GripperCommand_Request__Sequence * array);

/// Create array of srv/GripperCommand messages.
/**
 * It allocates the memory for the array and calls
 * catalyst_interfaces__srv__GripperCommand_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
catalyst_interfaces__srv__GripperCommand_Request__Sequence *
catalyst_interfaces__srv__GripperCommand_Request__Sequence__create(size_t size);

/// Destroy array of srv/GripperCommand messages.
/**
 * It calls
 * catalyst_interfaces__srv__GripperCommand_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Request__Sequence__destroy(catalyst_interfaces__srv__GripperCommand_Request__Sequence * array);

/// Check for srv/GripperCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Request__Sequence__are_equal(const catalyst_interfaces__srv__GripperCommand_Request__Sequence * lhs, const catalyst_interfaces__srv__GripperCommand_Request__Sequence * rhs);

/// Copy an array of srv/GripperCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Request__Sequence__copy(
  const catalyst_interfaces__srv__GripperCommand_Request__Sequence * input,
  catalyst_interfaces__srv__GripperCommand_Request__Sequence * output);

/// Initialize srv/GripperCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * catalyst_interfaces__srv__GripperCommand_Response
 * )) before or use
 * catalyst_interfaces__srv__GripperCommand_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Response__init(catalyst_interfaces__srv__GripperCommand_Response * msg);

/// Finalize srv/GripperCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Response__fini(catalyst_interfaces__srv__GripperCommand_Response * msg);

/// Create srv/GripperCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * catalyst_interfaces__srv__GripperCommand_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
catalyst_interfaces__srv__GripperCommand_Response *
catalyst_interfaces__srv__GripperCommand_Response__create();

/// Destroy srv/GripperCommand message.
/**
 * It calls
 * catalyst_interfaces__srv__GripperCommand_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Response__destroy(catalyst_interfaces__srv__GripperCommand_Response * msg);

/// Check for srv/GripperCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Response__are_equal(const catalyst_interfaces__srv__GripperCommand_Response * lhs, const catalyst_interfaces__srv__GripperCommand_Response * rhs);

/// Copy a srv/GripperCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Response__copy(
  const catalyst_interfaces__srv__GripperCommand_Response * input,
  catalyst_interfaces__srv__GripperCommand_Response * output);

/// Initialize array of srv/GripperCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * catalyst_interfaces__srv__GripperCommand_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Response__Sequence__init(catalyst_interfaces__srv__GripperCommand_Response__Sequence * array, size_t size);

/// Finalize array of srv/GripperCommand messages.
/**
 * It calls
 * catalyst_interfaces__srv__GripperCommand_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Response__Sequence__fini(catalyst_interfaces__srv__GripperCommand_Response__Sequence * array);

/// Create array of srv/GripperCommand messages.
/**
 * It allocates the memory for the array and calls
 * catalyst_interfaces__srv__GripperCommand_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
catalyst_interfaces__srv__GripperCommand_Response__Sequence *
catalyst_interfaces__srv__GripperCommand_Response__Sequence__create(size_t size);

/// Destroy array of srv/GripperCommand messages.
/**
 * It calls
 * catalyst_interfaces__srv__GripperCommand_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
void
catalyst_interfaces__srv__GripperCommand_Response__Sequence__destroy(catalyst_interfaces__srv__GripperCommand_Response__Sequence * array);

/// Check for srv/GripperCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Response__Sequence__are_equal(const catalyst_interfaces__srv__GripperCommand_Response__Sequence * lhs, const catalyst_interfaces__srv__GripperCommand_Response__Sequence * rhs);

/// Copy an array of srv/GripperCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_catalyst_interfaces
bool
catalyst_interfaces__srv__GripperCommand_Response__Sequence__copy(
  const catalyst_interfaces__srv__GripperCommand_Response__Sequence * input,
  catalyst_interfaces__srv__GripperCommand_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CATALYST_INTERFACES__SRV__DETAIL__GRIPPER_COMMAND__FUNCTIONS_H_
