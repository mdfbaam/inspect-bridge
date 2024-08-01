#pragma once

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/write.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class Write : ActionBase<inspect_bridge::action::Write> {
        public:
            using base_t = ActionBase<inspect_bridge::action::Write>;

            using base_t::base_t;

        protected:
            void execute(const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Write begins...");

                auto result = std::make_shared<typename base_t::action_t::Result>();

                result->point_cloud_out_path = "/mnt/c/tmp/out.pc";
                result->stl_out_path 	     = "/mnt/c/tmp/out.stl";

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }
    };
}
