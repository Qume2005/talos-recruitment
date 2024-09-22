#ifndef TURTLE_SPAWN_HPP
#define TURTLE_SPAWN_HPP

#include "turtle_spawn.hpp"
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/msg/pose.hpp>
#include <memory>
#include <chrono>

class TurtleSpawn {
public:
    TurtleSpawn(rclcpp::Node *node);
    void spawn_turtle(float x, float y, float theta, const std::string &name);
    bool wait_for_service();

private:
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Node::SharedPtr node_;
};

TurtleSpawn::TurtleSpawn(rclcpp::Node *node) 
    : node_(node) {
    spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("spawn");
}

bool TurtleSpawn::wait_for_service() {
        return spawn_client_->wait_for_service(std::chrono::seconds(1));
    }

void TurtleSpawn::spawn_turtle(float x, float y, float theta, const std::string &name) {
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;
    request->name = name;

    auto future = spawn_client_->async_send_request(request,
        [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
            try {
                future.get(); // 确保成功获取响应
                RCLCPP_INFO(node_->get_logger(), "Turtle spawned successfully.");
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to spawn turtle: %s", e.what());
            }
        });
}

#endif // TURTLE_SPAWN_HPP
