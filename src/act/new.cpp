// C++
#include <functional>
#include <memory>
#include <thread>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/new.hpp"

// Local
#include "inspect_bridge/libfmt_compat.hpp"

namespace ornl::ros::ib {
    class NewAction : public rclcpp::Node {
        public:
            using New           = inspect_bridge::action::New;
            using GoalHandleNew = rclcpp_action::ServerGoalHandle<New>;

            explicit NewAction(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("new_server", options) {
                using namespace std::placeholders;

                RCLCPP_FMT_INFO(this->get_logger(), "New action server starts...");

                m_server = rclcpp_action::create_server<New>(
                    this,
                    "new",
                    std::bind(&NewAction::enqueue, this, _1, _2),
                    std::bind(&NewAction::cancel,  this, _1),
                    std::bind(&NewAction::accept,  this, _1)
                );
            }

        protected:
            rclcpp_action::GoalResponse enqueue([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const New::Goal> goal) {
                RCLCPP_FMT_INFO(this->get_logger(), "Received goal request for new project '{}'", goal->scan_series_name);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse cancel(const std::shared_ptr<GoalHandleNew> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Cancel request for new project '{}'", handle->get_goal()->scan_series_name);
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void accept(const std::shared_ptr<GoalHandleNew> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Accepted request for new project '{}'", handle->get_goal()->scan_series_name);

                std::thread thread = std::thread(std::bind(&NewAction::execute, this, handle));
                thread.detach();
            }

            void execute(const std::shared_ptr<GoalHandleNew> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "New project '{}' creation begins...", handle->get_goal()->scan_series_name);

                auto result = std::make_shared<New::Result>();

                result->out_path = fmt::format("/mnt/c/ros/inspect/{}.atos", handle->get_goal()->scan_series_name);

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }

        private:
            rclcpp_action::Server<New>::SharedPtr m_server;
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ornl::ros::ib::NewAction>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
