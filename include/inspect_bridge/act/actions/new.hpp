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
            void execute(const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "New project '{}' creation begins...", handle->get_goal()->scan_series_name);

                auto result = std::make_shared<typename base_t::action_t::Result>();

                nlohmann::json request;

                request["command"]             = "new";
                request["arguments"]["series"] = handle->get_goal()->scan_series_name;

                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Sending INSPECT request; request = {}", request.dump(4));

                nlohmann::json response;

                try {
                    response = util::request(
                        m_node_ptr->get_parameter("inspect-hostname").as_string(),
                        m_node_ptr->get_parameter("inspect-port").as_string(),
                        request
                    );
                } catch(const std::exception& e) {
                    RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Bad response from INSPECT, exception caught. [e.what() = {}]", e.what());
                    handle->abort(result);
                    return;
                }

                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "INSPECT replied; response = {}", response.dump(4));

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }
    };
}
