#ifndef GST_PIPELINE__SYNTAX_SUGAR_H_
#define GST_PIPELINE__SYNTAX_SUGAR_H_

#include "rclcpp/rclcpp.hpp"

// parameter descriptions could really do with a constructor
rcl_interfaces::msg::ParameterDescriptor descr(
  const std::string & description, const bool & read_only = false,
  const std::string additional_constraints = "", const bool & dynamic_typing = false);

#endif  //GST_PIPELINE__SYNTAX_SUGAR_H_
