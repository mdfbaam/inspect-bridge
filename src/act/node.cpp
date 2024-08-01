// Local
#include "inspect_bridge/act/actions/new.hpp"
#include "inspect_bridge/act/actions/polygonize.hpp"
#include "inspect_bridge/act/actions/scan.hpp"
#include "inspect_bridge/act/actions/write.hpp"

namespace ornl::ros::ib {
    class ActionNode : public rclcpp::Node {
        public:
            explicit ActionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
                Node("inspect_action_server", options),
                m_actions {
                    actions::New(this, "new"),
                    actions::Scan(this, "scan"),
                    actions::Polygonize(this, "polygonize"),
                    actions::Write(this, "write")
                }
            {
                // NOP
            }

        protected:
            struct {
                actions::New newp;
                actions::Scan scan;
                actions::Polygonize polygonize;
                actions::Write write;
            } m_actions;
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ornl::ros::ib::ActionNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
