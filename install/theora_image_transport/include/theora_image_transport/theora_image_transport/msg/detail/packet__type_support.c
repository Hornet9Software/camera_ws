// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from theora_image_transport:msg/Packet.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "theora_image_transport/msg/detail/packet__rosidl_typesupport_introspection_c.h"
#include "theora_image_transport/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "theora_image_transport/msg/detail/packet__functions.h"
#include "theora_image_transport/msg/detail/packet__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  theora_image_transport__msg__Packet__init(message_memory);
}

void theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_fini_function(void * message_memory)
{
  theora_image_transport__msg__Packet__fini(message_memory);
}

size_t theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__size_function__Packet__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__get_const_function__Packet__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__get_function__Packet__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__fetch_function__Packet__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__get_const_function__Packet__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__assign_function__Packet__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__get_function__Packet__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__resize_function__Packet__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(theora_image_transport__msg__Packet, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(theora_image_transport__msg__Packet, data),  // bytes offset in struct
    NULL,  // default value
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__size_function__Packet__data,  // size() function pointer
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__get_const_function__Packet__data,  // get_const(index) function pointer
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__get_function__Packet__data,  // get(index) function pointer
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__fetch_function__Packet__data,  // fetch(index, &value) function pointer
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__assign_function__Packet__data,  // assign(index, value) function pointer
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__resize_function__Packet__data  // resize(index) function pointer
  },
  {
    "b_o_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(theora_image_transport__msg__Packet, b_o_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "e_o_s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(theora_image_transport__msg__Packet, e_o_s),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "granulepos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(theora_image_transport__msg__Packet, granulepos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "packetno",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(theora_image_transport__msg__Packet, packetno),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_members = {
  "theora_image_transport__msg",  // message namespace
  "Packet",  // message name
  6,  // number of fields
  sizeof(theora_image_transport__msg__Packet),
  theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_member_array,  // message members
  theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_init_function,  // function to initialize message memory (memory has to be allocated)
  theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_type_support_handle = {
  0,
  &theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_theora_image_transport
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, theora_image_transport, msg, Packet)() {
  theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_type_support_handle.typesupport_identifier) {
    theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &theora_image_transport__msg__Packet__rosidl_typesupport_introspection_c__Packet_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
