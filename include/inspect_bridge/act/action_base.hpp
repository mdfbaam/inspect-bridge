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
                    fmt::format("/inspect_bridge/{}", action_name),
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

            virtual void execute_pre_hook([[maybe_unused]] const std::shared_ptr<action_handle_t> handle) {
                RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "[with p_action_t = {}] Generic request begins...", util::demangle<action_t>());
            }

            /*!
             * \brief Hook that allows customization of the json request object to INSPECT.
             * \param handle: Handle for this action request.
             * \return Request json object.
             * \note The request object must have two keys - the "command" (string) and the "arguments" (object).
             *       These are passed directly to the
             */
            virtual nlohmann::json execute_pack_hook([[maybe_unused]] const std::shared_ptr<action_handle_t> handle) {
                nlohmann::json request;

                request["command"]   = "dummy";
                request["arguments"] = nlohmann::json::object();

                return request;
            }

            /*!
             * \brief Unpacks the result json object from INSPECT.
             * \param handle: Handle for this action request.
             * \param json_result: Result json representation from INSPECT.
             * \return The ROS result object.
             */
            virtual std::shared_ptr<typename action_t::Result> execute_unpack_hook(
                [[maybe_unused]] const std::shared_ptr<action_handle_t> handle,
                [[maybe_unused]] nlohmann::json json_result
            ) {
                auto result = std::make_shared<typename action_t::Result>();

                return result;
            }

            virtual void execute(const std::shared_ptr<action_handle_t> handle) {
                this->execute_pre_hook(handle);

                // Construct a request json.
                nlohmann::json request = this->execute_pack_hook(handle);
                nlohmann::json response;

                bool execept = false;

                std::shared_ptr<typename action_t::Result> result = nullptr;

                // Send the requst to INSPECT.
                if (!m_node_ptr->get_parameter("dummy").as_bool()) {
                    if (!request.empty()) {
                        RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Sending INSPECT request; request = {}", request.dump(4));
                        try {
                            response = util::request(
                                m_node_ptr->get_parameter("inspect-hostname").as_string(),
                                m_node_ptr->get_parameter("inspect-port").as_string(),
                                request
                            );
                        } catch(const std::exception& e) {
                            RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "Bad/no response from INSPECT, exception caught. [e.what() = {}]", e.what());
                            execept = true;
                        }

                        // If everything looks okay, try to unpack the response.
                        if (!execept) {
                            RCLCPP_FMT_INFO(m_node_ptr->get_logger(), "INSPECT replied; response = {}", response.dump(4));

                            if (response["status"] == true) {
                                result = this->execute_unpack_hook(handle, response["result"]);
                            }
                        }
                    }
                } else {
                    RCLCPP_WARN(m_node_ptr->get_logger(), "Dummy mode is enabled, not sending request to INSPECT.");
                    result = std::make_shared<typename action_t::Result>();
                }

                // Tell ROS the result.
                if (rclcpp::ok()) {
                    if (result == nullptr) {
                        result = std::make_shared<typename action_t::Result>();
                        handle->abort(result);
                        return;
                    }

                    handle->succeed(result);
                }
            }

            rclcpp::Node* m_node_ptr;
            typename rclcpp_action::Server<action_t>::SharedPtr m_server;
    };
}
