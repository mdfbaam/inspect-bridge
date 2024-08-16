#pragma once

// C++
#include <fstream>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/cad.hpp"

// Local
#include "inspect_bridge/act/action_base.hpp"

namespace ornl::ros::ib::actions {
    class Cad : ActionBase<inspect_bridge::action::Cad> {
        public:
            using base_t = ActionBase<inspect_bridge::action::Cad>;

            using base_t::base_t;

        protected:
            void execute_pre_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Cad reference import begins...");
            }

            nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<typename base_t::action_handle_t> handle) {
                nlohmann::json request;

                /*
                std::string stl_path = "/ros/stl/platypus Modified_noBeads.stl";
                std::ifstream stl_in(stl_path, std::ios::in | std::ios::binary);
                if(!stl_in.is_open()) {
                    RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Failed to load file - {}", stl_path);
                    return { };
                }
                std::string stl = std::string { std::istreambuf_iterator<char>(stl_in), std::istreambuf_iterator<char>() };

                std::string stl64 = base64::encode_into<std::string>(stl);
                */

                request["command"]   		  = "cad";
                request["arguments"]["stl64"] = handle->get_goal()->stl64;

                return request;
            }
    };
}
