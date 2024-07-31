#include <fmt/core.h>

#define RCLCPP_FMT_INFO(logger, ...) RCLCPP_INFO(logger, fmt::format(__VA_ARGS__).c_str())
