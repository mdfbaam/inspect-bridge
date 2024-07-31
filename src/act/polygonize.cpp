// C++
#include <functional>
#include <memory>
#include <thread>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/polygonize.hpp"

// Local
#include "inspect_bridge/libfmt_compat.hpp"

namespace ornl::ros::ib {
    class PolygonizeAction : public rclcpp::Node {
        public:
            using Polygonize     	   = inspect_bridge::action::Polygonize;
            using GoalHandlePolygonize = rclcpp_action::ServerGoalHandle<Polygonize>;

            explicit PolygonizeAction(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("polygonize_server", options) {
                using namespace std::placeholders;

                RCLCPP_FMT_INFO(this->get_logger(), "Polygonize action server starts...");

                m_server = rclcpp_action::create_server<Polygonize>(
                    this,
                    "polygonize",
                    std::bind(&PolygonizeAction::enqueue, this, _1, _2),
                    std::bind(&PolygonizeAction::cancel,  this, _1),
                    std::bind(&PolygonizeAction::accept,  this, _1)
                );
            }

        protected:
            rclcpp_action::GoalResponse enqueue([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const Polygonize::Goal> goal) {
                RCLCPP_FMT_INFO(this->get_logger(), "Received goal request for polygonize");
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse cancel([[maybe_unused]] const std::shared_ptr<GoalHandlePolygonize> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Cancel request for polygonize");
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void accept(const std::shared_ptr<GoalHandlePolygonize> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Accepted request for polygonize");

                std::thread thread = std::thread(std::bind(&PolygonizeAction::execute, this, handle));
                thread.detach();
            }

            void execute(const std::shared_ptr<GoalHandlePolygonize> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Polygonize begins...");

                auto result = std::make_shared<Polygonize::Result>();

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }

        private:
            rclcpp_action::Server<Polygonize>::SharedPtr m_server;
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ornl::ros::ib::PolygonizeAction>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
