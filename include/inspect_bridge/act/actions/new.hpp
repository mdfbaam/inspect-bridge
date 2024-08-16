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

                /*
                std::string package_share_directory = ament_index_cpp::get_package_share_directory("foundry_goms_control_loop");
                std::string template_path = package_share_directory + "/templates/pallet_1.zinspect";
                std::ifstream template_in(template_path, std::ios::in | std::ios::binary);
                if(!template_in.is_open()) {
                    RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Failed to load file - {}", template_path);
                    return { };
                }
                std::string template_s = std::string { std::istreambuf_iterator<char>(template_in), std::istreambuf_iterator<char>() };

                std::string template64 = base64::encode_into<std::string>(template_s);
                */

                request["command"]                 = "new";
                request["arguments"]["series"]     = handle->get_goal()->scan_series_name;
                request["arguments"]["template64"] = handle->get_goal()->template64;

                return request;
            }
    };
}
