#pragma once

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/new.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class New : ActionBase<inspect_bridge::action::New> {
        public:
            using base_t = ActionBase<inspect_bridge::action::New>;

            using base_t::base_t;

        protected:
            void execute_pre_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "New project '{}' creation begins...", handle->get_goal()->scan_series_name);
            }

            nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                nlohmann::json request;

                request["command"]                 = "new";
                request["arguments"]["series"]     = handle->get_goal()->scan_series_name;
                request["arguments"]["template64"] = "";

                return request;
            }
    };
}
