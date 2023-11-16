// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "kalman_interfaces/msg/detail/gps_tag_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace kalman_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GpsTagArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) kalman_interfaces::msg::GpsTagArray(_init);
}

void GpsTagArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<kalman_interfaces::msg::GpsTagArray *>(message_memory);
  typed_message->~GpsTagArray();
}

size_t size_function__GpsTagArray__tags(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<kalman_interfaces::msg::GpsTag> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GpsTagArray__tags(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<kalman_interfaces::msg::GpsTag> *>(untyped_member);
  return &member[index];
}

void * get_function__GpsTagArray__tags(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<kalman_interfaces::msg::GpsTag> *>(untyped_member);
  return &member[index];
}

void fetch_function__GpsTagArray__tags(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const kalman_interfaces::msg::GpsTag *>(
    get_const_function__GpsTagArray__tags(untyped_member, index));
  auto & value = *reinterpret_cast<kalman_interfaces::msg::GpsTag *>(untyped_value);
  value = item;
}

void assign_function__GpsTagArray__tags(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<kalman_interfaces::msg::GpsTag *>(
    get_function__GpsTagArray__tags(untyped_member, index));
  const auto & value = *reinterpret_cast<const kalman_interfaces::msg::GpsTag *>(untyped_value);
  item = value;
}

void resize_function__GpsTagArray__tags(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<kalman_interfaces::msg::GpsTag> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GpsTagArray_message_member_array[1] = {
  {
    "tags",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<kalman_interfaces::msg::GpsTag>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kalman_interfaces::msg::GpsTagArray, tags),  // bytes offset in struct
    nullptr,  // default value
    size_function__GpsTagArray__tags,  // size() function pointer
    get_const_function__GpsTagArray__tags,  // get_const(index) function pointer
    get_function__GpsTagArray__tags,  // get(index) function pointer
    fetch_function__GpsTagArray__tags,  // fetch(index, &value) function pointer
    assign_function__GpsTagArray__tags,  // assign(index, value) function pointer
    resize_function__GpsTagArray__tags  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GpsTagArray_message_members = {
  "kalman_interfaces::msg",  // message namespace
  "GpsTagArray",  // message name
  1,  // number of fields
  sizeof(kalman_interfaces::msg::GpsTagArray),
  GpsTagArray_message_member_array,  // message members
  GpsTagArray_init_function,  // function to initialize message memory (memory has to be allocated)
  GpsTagArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GpsTagArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GpsTagArray_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace kalman_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<kalman_interfaces::msg::GpsTagArray>()
{
  return &::kalman_interfaces::msg::rosidl_typesupport_introspection_cpp::GpsTagArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, kalman_interfaces, msg, GpsTagArray)() {
  return &::kalman_interfaces::msg::rosidl_typesupport_introspection_cpp::GpsTagArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
