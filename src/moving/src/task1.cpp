#include "turtle_controller.hpp"
#include <array>
#include <chrono>
#include <tuple>
#include "config.hpp"

template<size_t N>
class Task : public TurtleController<N> {
public:
    Task(std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N> targets)
    : TurtleController<N>("task1", targets) {  // 直接将参数传递给基类构造函数
        targets_ = targets;  // 初始化类内部变量
        RCLCPP_INFO(this->get_logger(), "Task1 instance created with targets.");
    }

private:
    std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N> targets_;
};

// 设置目标数量
constexpr size_t N = 9;

// 定义目标数组，包含位置和等待时间
constexpr std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N> targets = {
    std::make_tuple(MAX_BORDER, MAX_BORDER, 6.0, std::chrono::seconds(5)),
    std::make_tuple(MID_BORDER, MIN_BORDER, 6.0, std::chrono::seconds(0)),
    std::make_tuple(MIN_BORDER, MAX_BORDER, 6.0, std::chrono::seconds(5)),
    std::make_tuple(MAX_BORDER, MID_BORDER, 6.0, std::chrono::seconds(0)),
    std::make_tuple(MIN_BORDER, MIN_BORDER, 6.0, std::chrono::seconds(5)),
    std::make_tuple(MID_BORDER, MAX_BORDER, 6.0, std::chrono::seconds(0)),
    std::make_tuple(MAX_BORDER, MIN_BORDER, 6.0, std::chrono::seconds(5)),
    std::make_tuple(MIN_BORDER, MID_BORDER, 6.0, std::chrono::seconds(0)),
    std::make_tuple(MAX_BORDER, MAX_BORDER, 6.0, std::chrono::seconds(1))
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto turtle_controller_node = std::make_shared<Task<N>>(targets);
        RCLCPP_INFO(rclcpp::get_logger("Task1"), "Turtle Controller Node is running...");
        rclcpp::spin(turtle_controller_node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("Task1"), "Exception thrown: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("Task1"), "Unknown exception thrown.");
    }
    rclcpp::shutdown();
    return 0;
}
