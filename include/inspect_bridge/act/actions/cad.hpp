#pragma once

// C++
#include <fstream>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/cad.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class Cad : ActionBase<inspect_bridge::action::Cad> {
        public:
            using base_t = ActionBase<inspect_bridge::action::Cad>;

            using base_t::base_t;

        protected:
            void execute_pre_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Cad reference import begins...");
            }

            nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                nlohmann::json request;

                request["command"]   		  = "cad";
                request["arguments"]["stl64"] = handle->get_goal()->stl64;

                return request;
            }
    };
}
