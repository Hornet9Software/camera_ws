// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from theora_image_transport:msg/Packet.idl
// generated code does not contain a copyright notice

#ifndef THEORA_IMAGE_TRANSPORT__MSG__DETAIL__PACKET__STRUCT_H_
#define THEORA_IMAGE_TRANSPORT__MSG__DETAIL__PACKET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Packet in the package theora_image_transport.
/**
  * ROS message adaptation of the ogg_packet struct from libogg,
  * see http://www.xiph.org/ogg/doc/libogg/ogg_packet.html.
 */
typedef struct theora_image_transport__msg__Packet
{
  /// Original sensor_msgs/Image header
  std_msgs__msg__Header header;
  /// Raw Theora packet data (combines packet and bytes fields from ogg_packet)
  rosidl_runtime_c__uint8__Sequence data;
  /// Flag indicating whether this packet begins a logical bitstream
  int32_t b_o_s;
  /// Flag indicating whether this packet ends a bitstream
  int32_t e_o_s;
  /// A number indicating the position of this packet in the decoded data
  int64_t granulepos;
  /// Sequential number of this packet in the ogg bitstream
  int64_t packetno;
} theora_image_transport__msg__Packet;

// Struct for a sequence of theora_image_transport__msg__Packet.
typedef struct theora_image_transport__msg__Packet__Sequence
{
  theora_image_transport__msg__Packet * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} theora_image_transport__msg__Packet__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THEORA_IMAGE_TRANSPORT__MSG__DETAIL__PACKET__STRUCT_H_
