#pragma once

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/new.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"
#include "inspect_bridge/util/paths.hpp"

namespace ornl::ros::ib::actions {
    class New : ActionBase<inspect_bridge::action::New> {
        public:
            using base_t = ActionBase<inspect_bridge::action::New>;

            using base_t::base_t;

        protected:
            void execute(const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "New project '{}' creation begins...", handle->get_goal()->scan_series_name);

                auto result = std::make_shared<typename base_t::action_t::Result>();

                std::string path    = fmt::format("/mnt/c/ros/inspect/{}/", handle->get_goal()->scan_series_name);
                std::string project = path + "project.zinspect";

                nlohmann::json request;

                request["command"]           = "new";
                request["arguments"]["path"] = util::wsl_to_win_path(project);

                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Sending INSPECT request; request = {}", request.dump(4));

                nlohmann::json response = util::request(
                    m_node_ptr->get_parameter("inspect-hostname").as_string(),
                    m_node_ptr->get_parameter("inspect-port").as_string(),
                    request
                );
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "INSPECT replied; response = {}", response.dump(4));

                result->out_path = path;

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }
    };
}
