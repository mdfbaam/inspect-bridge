#pragma once

// fmt
#include <fmt/core.h>

// Local
#include "inspect_bridge/util/system.hpp"

namespace ornl::ros::ib::util {
    // Pretend that I actually bothered to convert these paths
    std::string win_to_wsl_path(const std::string& win_path) {
        std::string result = util::exec(fmt::format("wslpath -u {}", win_path));

        return result;
    }

    std::string wsl_to_win_path(const std::string& wsl_path) {
        std::string result = util::exec(fmt::format("wslpath -w {}", wsl_path));

        return result;
    }
}
