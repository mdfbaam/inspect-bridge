// ROS2
#include <rclcpp/rclcpp.hpp>

// Local
#include <inspect_bridge/util/libfmt_compat.hpp>

namespace ornl::ros::ib {
    class ScanCoordinateNode : public rclcpp::Node {
        public:
            explicit ScanCoordinateNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
                Node("inspect_scan_coordinate_node", options)
            {
                RCLCPP_FMT_INFO(this->get_logger(), "INSPECT bridge scan coordinate subscriber now available");
            }

        protected:
    };
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ornl::ros::ib::ScanCoordinateNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
