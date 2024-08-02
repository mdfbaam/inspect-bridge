#pragma once

// C++
#include <fstream>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/write.hpp"

// Contrib
#include "inspect_bridge/contrib/base64.hpp"

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

                nlohmann::json request;

                request["command"]             = "write";
                request["arguments"]["series"] = handle->get_goal()->scan_series_name;

                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Sending INSPECT request; request = {}", request.dump(4));

                nlohmann::json response = util::request(
                    m_node_ptr->get_parameter("inspect-hostname").as_string(),
                    m_node_ptr->get_parameter("inspect-port").as_string(),
                    request
                );

                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "INSPECT replied; response truncated");

                std::vector<char> stl_data = base64::decode_into<std::vector<char>>(std::string(response["result"]["stl64"]));
                std::string asc_data       = base64::decode_into<std::string>(std::string(response["result"]["asc64"]));

                std::ofstream stl_out("/home/gom/stl_out.stl", std::ios::out | std::ios::binary);
                stl_out.write(reinterpret_cast<const char*>(stl_data.data()), stl_data.size());
                stl_out.close();

                std::ofstream asc_out("/home/gom/asc_out.asc", std::ios::out | std::ios::binary);
                asc_out.write(reinterpret_cast<const char*>(asc_data.data()), asc_data.size());
                asc_out.close();

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }
    };
}
