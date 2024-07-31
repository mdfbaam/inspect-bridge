// C++
#include <functional>
#include <memory>
#include <thread>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ROS 2 Interfaces (Generated)
#include "inspect_bridge/action/write.hpp"

// Local
#include "inspect_bridge/libfmt_compat.hpp"

namespace ornl::ros::ib {
    class WriteAction : public rclcpp::Node {
        public:
            using Write           = inspect_bridge::action::Write;
            using GoalHandleWrite = rclcpp_action::ServerGoalHandle<Write>;

            explicit WriteAction(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("write_server", options) {
                using namespace std::placeholders;

                RCLCPP_FMT_INFO(this->get_logger(), "Write action server starts...");

                m_server = rclcpp_action::create_server<Write>(
                    this,
                    "write",
                    std::bind(&WriteAction::enqueue, this, _1, _2),
                    std::bind(&WriteAction::cancel,  this, _1),
                    std::bind(&WriteAction::accept,  this, _1)
                );
            }

        protected:
            rclcpp_action::GoalResponse enqueue([[maybe_unused]] const rclcpp_action::GoalUUID& uuid, [[maybe_unused]] std::shared_ptr<const Write::Goal> goal) {
                RCLCPP_FMT_INFO(this->get_logger(), "Received goal request for write");
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse cancel([[maybe_unused]] const std::shared_ptr<GoalHandleWrite> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Cancel request for project write");
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void accept(const std::shared_ptr<GoalHandleWrite> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Accepted request for project write");

                std::thread thread = std::thread(std::bind(&WriteAction::execute, this, handle));
                thread.detach();
            }

            void execute(const std::shared_ptr<GoalHandleWrite> handle) {
                RCLCPP_FMT_INFO(this->get_logger(), "Write begins...");

                auto result = std::make_shared<Write::Result>();

                result->point_cloud_out_path = "/mnt/c/tmp/out.pc";
                result->stl_out_path 	     = "/mnt/c/tmp/out.stl";

                if (rclcpp::ok()) {
                    handle->succeed(result);
                }
            }

        private:
            rclcpp_action::Server<Write>::SharedPtr m_server;
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ornl::ros::ib::WriteAction>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
