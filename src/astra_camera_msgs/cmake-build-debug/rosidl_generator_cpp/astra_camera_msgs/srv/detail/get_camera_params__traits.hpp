// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astra_camera_msgs:srv/GetCameraParams.idl
// generated code does not contain a copyright notice

#ifndef ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_PARAMS__TRAITS_HPP_
#define ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_PARAMS__TRAITS_HPP_

#include "astra_camera_msgs/srv/detail/get_camera_params__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const astra_camera_msgs::srv::GetCameraParams_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const astra_camera_msgs::srv::GetCameraParams_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<astra_camera_msgs::srv::GetCameraParams_Request>()
{
  return "astra_camera_msgs::srv::GetCameraParams_Request";
}

template<>
inline const char * name<astra_camera_msgs::srv::GetCameraParams_Request>()
{
  return "astra_camera_msgs/srv/GetCameraParams_Request";
}

template<>
struct has_fixed_size<astra_camera_msgs::srv::GetCameraParams_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<astra_camera_msgs::srv::GetCameraParams_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<astra_camera_msgs::srv::GetCameraParams_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const astra_camera_msgs::srv::GetCameraParams_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: l_intr_p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.l_intr_p.size() == 0) {
      out << "l_intr_p: []\n";
    } else {
      out << "l_intr_p:\n";
      for (auto item : msg.l_intr_p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: r_intr_p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.r_intr_p.size() == 0) {
      out << "r_intr_p: []\n";
    } else {
      out << "r_intr_p:\n";
      for (auto item : msg.r_intr_p) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: r2l_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.r2l_r.size() == 0) {
      out << "r2l_r: []\n";
    } else {
      out << "r2l_r:\n";
      for (auto item : msg.r2l_r) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: r2l_t
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.r2l_t.size() == 0) {
      out << "r2l_t: []\n";
    } else {
      out << "r2l_t:\n";
      for (auto item : msg.r2l_t) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: l_k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.l_k.size() == 0) {
      out << "l_k: []\n";
    } else {
      out << "l_k:\n";
      for (auto item : msg.l_k) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: r_k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.r_k.size() == 0) {
      out << "r_k: []\n";
    } else {
      out << "r_k:\n";
      for (auto item : msg.r_k) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
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

inline std::string to_yaml(const astra_camera_msgs::srv::GetCameraParams_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<astra_camera_msgs::srv::GetCameraParams_Response>()
{
  return "astra_camera_msgs::srv::GetCameraParams_Response";
}

template<>
inline const char * name<astra_camera_msgs::srv::GetCameraParams_Response>()
{
  return "astra_camera_msgs/srv/GetCameraParams_Response";
}

template<>
struct has_fixed_size<astra_camera_msgs::srv::GetCameraParams_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astra_camera_msgs::srv::GetCameraParams_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astra_camera_msgs::srv::GetCameraParams_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<astra_camera_msgs::srv::GetCameraParams>()
{
  return "astra_camera_msgs::srv::GetCameraParams";
}

template<>
inline const char * name<astra_camera_msgs::srv::GetCameraParams>()
{
  return "astra_camera_msgs/srv/GetCameraParams";
}

template<>
struct has_fixed_size<astra_camera_msgs::srv::GetCameraParams>
  : std::integral_constant<
    bool,
    has_fixed_size<astra_camera_msgs::srv::GetCameraParams_Request>::value &&
    has_fixed_size<astra_camera_msgs::srv::GetCameraParams_Response>::value
  >
{
};

template<>
struct has_bounded_size<astra_camera_msgs::srv::GetCameraParams>
  : std::integral_constant<
    bool,
    has_bounded_size<astra_camera_msgs::srv::GetCameraParams_Request>::value &&
    has_bounded_size<astra_camera_msgs::srv::GetCameraParams_Response>::value
  >
{
};

template<>
struct is_service<astra_camera_msgs::srv::GetCameraParams>
  : std::true_type
{
};

template<>
struct is_service_request<astra_camera_msgs::srv::GetCameraParams_Request>
  : std::true_type
{
};

template<>
struct is_service_response<astra_camera_msgs::srv::GetCameraParams_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ASTRA_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_PARAMS__TRAITS_HPP_
