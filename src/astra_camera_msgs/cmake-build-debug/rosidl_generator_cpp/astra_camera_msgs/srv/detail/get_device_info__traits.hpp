// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astra_camera_msgs:srv/GetDeviceInfo.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__TRAITS_HPP_
#define ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__TRAITS_HPP_

#include "astra_camera_msgs/srv/detail/get_device_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const astra_camera_msgs::srv::GetDeviceInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const astra_camera_msgs::srv::GetDeviceInfo_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<astra_camera_msgs::srv::GetDeviceInfo_Request>()
{
  return "astra_camera_msgs::srv::GetDeviceInfo_Request";
}

template<>
inline const char * name<astra_camera_msgs::srv::GetDeviceInfo_Request>()
{
  return "astra_camera_msgs/srv/GetDeviceInfo_Request";
}

template<>
struct has_fixed_size<astra_camera_msgs::srv::GetDeviceInfo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astra_camera_msgs::srv::GetDeviceInfo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astra_camera_msgs::srv::GetDeviceInfo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "astra_camera_msgs/msg/detail/device_info__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const astra_camera_msgs::srv::GetDeviceInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_yaml(msg.info, out, indentation + 2);
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const astra_camera_msgs::srv::GetDeviceInfo_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<astra_camera_msgs::srv::GetDeviceInfo_Response>()
{
  return "astra_camera_msgs::srv::GetDeviceInfo_Response";
}

template<>
inline const char * name<astra_camera_msgs::srv::GetDeviceInfo_Response>()
{
  return "astra_camera_msgs/srv/GetDeviceInfo_Response";
}

template<>
struct has_fixed_size<astra_camera_msgs::srv::GetDeviceInfo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astra_camera_msgs::srv::GetDeviceInfo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astra_camera_msgs::srv::GetDeviceInfo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astra_camera_msgs::srv::GetDeviceInfo>()
{
  return "astra_camera_msgs::srv::GetDeviceInfo";
}

template<>
inline const char * name<astra_camera_msgs::srv::GetDeviceInfo>()
{
  return "astra_camera_msgs/srv/GetDeviceInfo";
}

template<>
struct has_fixed_size<astra_camera_msgs::srv::GetDeviceInfo>
  : std::integral_constant<
    bool,
    has_fixed_size<astra_camera_msgs::srv::GetDeviceInfo_Request>::value &&
    has_fixed_size<astra_camera_msgs::srv::GetDeviceInfo_Response>::value
  >
{
};

template<>
struct has_bounded_size<astra_camera_msgs::srv::GetDeviceInfo>
  : std::integral_constant<
    bool,
    has_bounded_size<astra_camera_msgs::srv::GetDeviceInfo_Request>::value &&
    has_bounded_size<astra_camera_msgs::srv::GetDeviceInfo_Response>::value
  >
{
};

template<>
struct is_service<astra_camera_msgs::srv::GetDeviceInfo>
  : std::true_type
{
};

template<>
struct is_service_request<astra_camera_msgs::srv::GetDeviceInfo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astra_camera_msgs::srv::GetDeviceInfo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_DEVICE_INFO__TRAITS_HPP_
