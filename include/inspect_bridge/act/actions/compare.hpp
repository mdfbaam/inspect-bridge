#pragma once

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/empty.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class Compare : ActionBase<inspect_bridge::action::Empty> {
        public:
            using base_t = ActionBase<inspect_bridge::action::Empty>;

            using base_t::base_t;

        protected:
            void execute_pre_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "CAD comparison begins...");
            }

            nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                nlohmann::json request;

                request["command"]   = "compare";
                request["arguments"] = nlohmann::json::object();

                return request;
            }
    };
}
