#pragma once

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/scan.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class Scan : ActionBase<inspect_bridge::action::Scan> {
        public:
            using base_t = ActionBase<inspect_bridge::action::Scan>;

            using base_t::base_t;

        protected:
            void execute_pre_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Scan #{} begins...", handle->get_goal()->scan_id);
            }

            nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                nlohmann::json request;

                request["command"]   = "scan";
                request["arguments"] = nlohmann::json::object();

                return request;
            }
    };
}
