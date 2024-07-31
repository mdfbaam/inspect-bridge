// C++
#include <functional>
#include <memory>
#include <thread>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/scan.hpp"

// Local
#include "inspect_bridge/libfmt_compat.hpp"

namespace ornl::ros::ib {
    class ScanAction : public rclcpp::Node {
        public:
            using Scan           = inspect_bridge::action::Scan;
            using GoalHandleScan = rclcpp_action::ServerGoalHandle<Scan>;

            explicit ScanAction(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("scan_server", options) {
                using namespace std::placeholders;

                RCLCPP_FMT_INFO(this->get_logger(), "Scan action server starts...");

                m_server = rclcpp_action::create_server<Scan>(
                    this,
                    "scan",
                    std::bind(&ScanAction::enqueue, this, _1, _2),
                    std::bind(&ScanAction::cancel,  this, _1),
                    std::bind(&ScanAction::accept,  this, _1)
                );
            }

        protected:
            rclcpp_action::GoalResponse enqueue([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Scan::Goal> goal) {
                RCLCPP_FMT_INFO(this->get_logger(), "Received goal request for scan #{}", goal->scan_id);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse cancel(const std::shared_ptr<GoalHandleScan> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Cancel request for scan #{}", handle->get_goal()->scan_id);
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void accept(const std::shared_ptr<GoalHandleScan> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Accepted request for scan #{}", handle->get_goal()->scan_id);

                std::thread thread = std::thread(std::bind(&ScanAction::execute, this, handle));
                thread.detach();
            }

            void execute(const std::shared_ptr<GoalHandleScan> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Scan #{} begins...", handle->get_goal()->scan_id);

                auto result = std::make_shared<Scan::Result>();

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }

        private:
            rclcpp_action::Server<Scan>::SharedPtr m_server;
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ornl::ros::ib::ScanAction>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
