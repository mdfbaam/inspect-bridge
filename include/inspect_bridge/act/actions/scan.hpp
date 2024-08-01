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
            void execute(const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Scan #{} begins...", handle->get_goal()->scan_id);

                auto result = std::make_shared<typename base_t::action_t::Result>();

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }
    };
}
