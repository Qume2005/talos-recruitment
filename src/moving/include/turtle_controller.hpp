#ifndef TURTLE_CONTROLLER_HPP
#define TURTLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "target_manager.hpp"
#include "turtle_mover.hpp"
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include "config.hpp"
#include "turtle_spawn.hpp"
#include "turtle_id_generator.hpp"
#include "turtle_state.hpp"
#include "turtle_pen.hpp"

template<size_t N>
class TurtleController : public rclcpp::Node {
public:
    TurtleController(
        const std::string name, 
        const std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N>& targets
    ) :
        Node(name),
        has_reached_(false),
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
        wait_timer_(nullptr),
        targets_(targets),
        current_target_iter_(targets_.begin()), 
        state_manager_(),
        id_generator_(),
        turtle_mover_(),
        turtle_spawn_(this),
        turtle_pen_(this)
    {   
        // 等待 turtlesim 相关服务连接
        wait_for_services();
        turtle_pen_.set_pen(false); // 默认关闭画笔
    }

    void set_targets(std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N> targets);

private:
    std::atomic<bool> has_reached_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr wait_timer_;
    turtlesim::msg::Pose current_pose_;
    TargetManager<N> targets_;
    typename decltype(targets_.targets_)::const_iterator current_target_iter_;
    TurtleStateManager state_manager_;
    TurtleIdGenerator id_generator_;
    TurtleMover turtle_mover_;
    TurtleSpawn turtle_spawn_;
    TurtlePen turtle_pen_;

    void wait_for_services();
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
    void timer_callback();
    void wait_callback();
    inline double target_x() const { return std::get<0>(*current_target_iter_); }
    inline double target_y() const { return std::get<1>(*current_target_iter_); }
};

template<size_t N>
void TurtleController<N>::wait_for_services() {
    RCLCPP_INFO(this->get_logger(), "Waiting for turtlesim services to become available...");
    // 等待 turtlesim 相关服务连接
    while (!turtle_pen_.wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the services. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Still waiting for turtlesim services...");
    }

    RCLCPP_INFO(this->get_logger(), "TurtleController instance created");
}

template<size_t N>
void TurtleController<N>::set_targets(std::array<std::tuple<double, double, double, std::chrono::duration<int64_t>>, N> targets) {
    targets_ = targets;
    current_target_iter_ = targets_.begin(); // 初始化迭代器
}

template<size_t N>
void TurtleController<N>::pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
}

template<size_t N>
void TurtleController<N>::timer_callback() {
    if (state_manager_.get_state() == TurtleState::WAITING) return;

    geometry_msgs::msg::Twist move_cmd = turtle_mover_.calculate_movement(current_pose_, *current_target_iter_);
    cmd_vel_publisher_->publish(move_cmd); 

    if (move_cmd.linear.x == 0 && move_cmd.angular.z == 0 && current_target_iter_ != targets_.end()) {
        wait_timer_ = create_wall_timer(std::get<3>(*current_target_iter_), std::bind(&TurtleController::wait_callback, this));
        RCLCPP_INFO(this->get_logger(), "Reached target (%.2f, %.2f), waiting...", target_x(), target_y());
        ++current_target_iter_;
        state_manager_.set_state(TurtleState::WAITING); 
        wait_timer_->reset(); 
    }
}

template<size_t N>
void TurtleController<N>::wait_callback() {
    if (current_target_iter_ == targets_.end()) {
        RCLCPP_INFO(this->get_logger(), "All targets reached.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(this->get_logger(), "Shutting down...");
        timer_->cancel();
        rclcpp::shutdown();
        return; 
    }
    if (!has_reached_) { 
        has_reached_ = true;
        turtle_pen_.set_pen(true);
    }
    if (state_manager_.get_state() == TurtleState::WAITING) { 
        turtle_spawn_.spawn_turtle(current_pose_.x, current_pose_.y, current_pose_.theta, id_generator_.get());
        state_manager_.set_state(TurtleState::MOVING); 
        wait_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Waiting time finished, moving to the next target.");
    }
}

#endif // TURTLE_CONTROLLER_HPP
