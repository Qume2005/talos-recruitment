#include "rclcpp/rclcpp.hpp"
#include "turtle_controller.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto turtle_controller_node = std::make_shared<TurtleController>();
        rclcpp::spin(turtle_controller_node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception thrown: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

