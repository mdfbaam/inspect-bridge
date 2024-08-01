#pragma once

// C++
#include <functional>
#include <memory>
#include <thread>

// json
#include <nlohmann/json.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Local
#include "inspect_bridge/util/libfmt_compat.hpp"
#include "inspect_bridge/util/demangle.hpp"
#include "inspect_bridge/util/network.hpp"

namespace ornl::ros::ib {
    template<typename p_action_t>
    class ActionBase {
        public:
            using this_t          = ActionBase<p_action_t>;

            using action_t        = p_action_t;
            using action_handle_t = rclcpp_action::ServerGoalHandle<action_t>;

            ActionBase(rclcpp::Node* node_ptr, std::string action_name) {
                using namespace std::placeholders;

                m_node_ptr = node_ptr;
                m_server = rclcpp_action::create_server<action_t>(
                    m_node_ptr,
                    action_name,
                    std::bind(&this_t::enqueue, this, _1, _2),
                    std::bind(&this_t::cancel,  this, _1),
                    std::bind(&this_t::accept,  this, _1)
                );
            }

        protected:
            virtual rclcpp_action::GoalResponse enqueue([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const typename action_t::Goal> goal) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "[with p_action_t = {}] Received goal request for generic action", util::demangle<action_t>());
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            virtual rclcpp_action::CancelResponse cancel([[maybe_unused]] const std::shared_ptr<action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "[with p_action_t = {}] Cancel request for generic action", util::demangle<action_t>());
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            virtual void accept(const std::shared_ptr<action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "[with p_action_t = {}] Accepted request for generic action", util::demangle<action_t>());

                std::thread thread = std::thread(std::bind(&this_t::execute, this, handle));
                thread.detach();
            }

            virtual void execute(const std::shared_ptr<action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "[with p_action_t = {}] Generic request begins...", util::demangle<action_t>());

                auto result = std::make_shared<typename action_t::Result>();

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }

            rclcpp::Node* m_node_ptr;
            typename rclcpp_action::Server<action_t>::SharedPtr m_server;
    };
}
