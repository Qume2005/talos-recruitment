#include "turtle_controller.hpp"

#define target_x std::get<0>(*current_target_iter_)
#define target_y std::get<1>(*current_target_iter_)

void TurtleController::spawn_turtle(float x, float y, float theta, const std::string &name) {
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = name;

    // 异步发送请求并处理响应
    auto future = spawn_client_->async_send_request(request,
        [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
            try {
                // 获取响应
                future.get();
                RCLCPP_INFO(this->get_logger(), "Turtle spawned at: (%.2f,%.2f).", current_pose_.x, current_pose_.y);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle: %s", e.what());
            }
        });
}

void TurtleController::pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
}

void TurtleController::timer_callback() {
    if (state_ == TurtleState::WAITING) return;

    if (current_target_iter_ == TARGET_MANAGER.end()) {
        RCLCPP_INFO(this->get_logger(), "All targets reached.");
        rclcpp::shutdown();
        return; 
    }

    geometry_msgs::msg::Twist move_cmd = mover_.calculate_movement(current_pose_, *current_target_iter_);

    cmd_vel_publisher_->publish(move_cmd); 

    if (move_cmd.linear.x == 0 && move_cmd.angular.z == 0) {
        RCLCPP_INFO(this->get_logger(), "Reached target (%.2f, %.2f), waiting...", target_x, target_y);
        spawn_turtle(current_pose_.x, current_pose_.y, current_pose_.theta, id_generator_.get());
        ++current_target_iter_;
        state_ = TurtleState::WAITING; 
        wait_timer_->reset(); 
    }
}

void TurtleController::wait_callback() {
    if (state_ == TurtleState::WAITING) {
        state_ = TurtleState::MOVING; 
        RCLCPP_INFO(this->get_logger(), "Waiting time finished, moving to the next target.");
    }
}