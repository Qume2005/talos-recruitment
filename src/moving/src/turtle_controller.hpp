#ifndef TURTLE_CONTROLLER_HPP
#define TURTLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "target_manager.hpp"
#include "turtle_mover.hpp"
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/msg/pose.hpp>
#include <atomic>
#include <chrono>

#define WAIT_TIME_SECONDS std::chrono::seconds(5) // 乌龟等待时间

enum class TurtleState {
    MOVING,
    WAITING
};

class TurtleIdGenerator {
public:
    TurtleIdGenerator() : current_id_(2) {}
    std::string get() { return "turtle" + std::to_string(current_id_++); }

private:
    std::atomic<int> current_id_;
};

class TurtleController : public rclcpp::Node {
public:
    TurtleController(): 
        Node("turtle_controller"),
        spawn_client_(create_client<turtlesim::srv::Spawn>("spawn")),
        pose_subscription_(create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1)
        )),
        cmd_vel_publisher_(create_publisher<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel",
            10
        )),
        timer_(create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&TurtleController::timer_callback, this)
        )),
        wait_timer_(create_wall_timer(
            WAIT_TIME_SECONDS,
            std::bind(&TurtleController::wait_callback, this)
        )),
        current_target_iter_(TARGET_MANAGER.begin()), 
        state_(TurtleState::MOVING), 
        id_generator_(), 
        mover_(1.5, 0.1) // LINEAR_SPEED and TOLERANCE
    {
        wait_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "TurtleController instance created");
    }

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr wait_timer_;

    turtlesim::msg::Pose current_pose_;
    TargetManager TARGET_MANAGER;
    decltype(TargetManager::TARGETS)::const_iterator current_target_iter_;
    TurtleState state_;
    TurtleIdGenerator id_generator_;
    TurtleMover mover_;

    void spawn_turtle(float x, float y, float theta, const std::string &name);
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
    void timer_callback();
    void wait_callback();
    inline double target_x() const;
    inline double target_y() const;
};

#endif // TURTLE_CONTROLLER_HPP
