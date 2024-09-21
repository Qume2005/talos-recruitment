#include "rclcpp/rclcpp.hpp"
#include "turtle_controller.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto turtle_controller_node = std::make_shared<TurtleController>(); //创建Node节点
        RCLCPP_INFO(rclcpp::get_logger("TurtleController"), "Turtle Controller Node is running...");
        rclcpp::spin(turtle_controller_node); //事件循环
    } catch (const std::exception &e) { //错误处理
        RCLCPP_ERROR(rclcpp::get_logger("TurtleController"), "Exception thrown: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("TurtleController"), "Unknown exception thrown.");
    }
    rclcpp::shutdown(); //关闭Node节点
    return 0;
}
