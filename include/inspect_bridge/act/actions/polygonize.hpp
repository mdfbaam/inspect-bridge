#pragma once

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/polygonize.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class Polygonize : ActionBase<inspect_bridge::action::Polygonize> {
        public:
            using base_t = ActionBase<inspect_bridge::action::Polygonize>;

            using base_t::base_t;

        protected:
            void execute(const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Polygonize begins...");

                auto result = std::make_shared<typename base_t::action_t::Result>();

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }
    };
}
