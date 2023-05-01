
#include <syntax_sugar.h>

// parameter descriptions could really do with a constructor
rcl_interfaces::msg::ParameterDescriptor descr(
  const std::string& description,
  const bool& read_only,
  const std::string additional_constraints,
  const bool& dynamic_typing)
{
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = description;
    descr.read_only = read_only;
    descr.additional_constraints = additional_constraints;
    descr.dynamic_typing = dynamic_typing;
    return descr;
}
