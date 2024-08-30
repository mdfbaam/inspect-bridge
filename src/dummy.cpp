// Local
#include "inspect_bridge/act/actions/cad.hpp"
#include "inspect_bridge/act/actions/calibrate.hpp"
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
                    actions::Cad(this, "cad"),
                    actions::Calibrate(this, "calibrate"),
                    actions::New(this, "new"),
                    actions::Scan(this, "scan"),
                    actions::Polygonize(this, "polygonize"),
                    actions::Write(this, "write")
                }
            {
                this->declare_parameter("inspect-hostname", "192.168.1.59");
                this->declare_parameter("inspect-port",     "8901");
                this->declare_parameter("dummy",            true);

                RCLCPP_FMT_INFO(this->get_logger(), "INSPECT bridge action server now available");
            }

        protected:
            struct {
                actions::Cad cad;
                actions::Calibrate calibrate;
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
