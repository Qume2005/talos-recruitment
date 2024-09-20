#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <chrono>
#include <array>
#include <vector>

using namespace std::chrono_literals;

constexpr std::array<std::array<double, 2>, 4> TARGETS = {
    std::array<double, 2>{16.5, 16.0},
    std::array<double, 2>{1.5, 16.5},
    std::array<double, 2>{1.0, 1.5},
    std::array<double, 2>{16.5, 1.0},
};
constexpr double LINEAR_SPEED = 1.0; // 线速度
constexpr double TOLERANCE = 0.1; // 到达目标位置的容差

class TurtleIdGenerator {
public:
    TurtleIdGenerator() : current_id_(2) {} // 从2开始生成ID
    std::string get() { return "turtle" + std::to_string(current_id_++); }
private:
    std::atomic<int> current_id_; // 线程安全的自增
};

class TurtleController : public rclcpp::Node
{
public:
    TurtleController()
        : Node("turtle_controller"), id_generator_(), current_target_index_(0)
    {
        // 设置服务客户端
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");

        // 订阅 turtle1 的位置
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));

        // 创建命令发布器
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // 定时器控制移动
        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleController::timer_callback, this));
    }

private:
    TurtleIdGenerator id_generator_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_; // 将命令发布器移到类成员
    
    rclcpp::TimerBase::SharedPtr timer_;
    turtlesim::msg::Pose current_pose_; // 当前乌龟的位置
    size_t current_target_index_; // 当前目标点索引

    void spawn_turtle(double x, double y, double theta, const std::string &name)
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = name;

        auto result_future = spawn_client_->async_send_request(request);
        if (result_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) 
        {
            if (result_future.valid()) {
                RCLCPP_INFO(this->get_logger(), "Spawned new turtle %s at (%.2f, %.2f)", name.c_str(), x, y);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle: future invalid");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service spawn: timeout waiting for response");
        }
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg; // 更新当前乌龟的姿态
    }

    void timer_callback()
    {
        if (current_target_index_ >= TARGETS.size()) {
            RCLCPP_INFO(this->get_logger(), "All targets reached.");
            rclcpp::shutdown();
            return; // 全部目标已到达，停止执行
        }

        double target_x = TARGETS[current_target_index_][0];
        double target_y = TARGETS[current_target_index_][1];

        // 计算与目标点的距离
        double distance_s = std::sqrt(std::pow(current_pose_.x - target_x, 2) + std::pow(current_pose_.y - target_y, 2));

        // 计算与目标的角度
        double angle_to_target = std::atan2(target_y - current_pose_.y, target_x - current_pose_.x);
        double angle_diff = normalize_angle(angle_to_target - current_pose_.theta); // 角度差

        // 控制运动
        auto move_cmd = geometry_msgs::msg::Twist();
        move_cmd.linear.x = LINEAR_SPEED;
        move_cmd.angular.z = 3 * angle_diff; // 确保乌龟朝向目标

        // 如果距离目标位置小于容差，生成新乌龟并切换到下一个目标点
        if (distance_s < TOLERANCE)
        {
            RCLCPP_INFO(this->get_logger(), "Reached target (%.2f, %.2f)", target_x, target_y);
            spawn_turtle(target_x, target_y, current_pose_.theta, id_generator_.get());
            current_target_index_++; // 更新目标点索引
        }

        cmd_vel_publisher_->publish(move_cmd); // 每次都发布运动命令
    }

    // 归一化角度，保持在 -π 到 π 之间
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}