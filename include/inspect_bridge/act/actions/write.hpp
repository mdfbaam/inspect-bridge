#pragma once

// C++
#include <fstream>

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
            void execute_pre_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Write begins...");
            }

            nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                nlohmann::json request;

                request["command"]             = "write";
                request["arguments"]["series"] = handle->get_goal()->scan_series_name;

                return request;
            }

            virtual std::shared_ptr<typename base_t::action_t::Result> execute_unpack_hook(
                [[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle,
                [[maybe_unused]] nlohmann::json json_result
            ) {
                auto result = std::make_shared<typename base_t::action_t::Result>();

                result->stl64 = json_result["stl64"];
                result->asc64 = json_result["asc64"];

                return result;
            }
    };
}
