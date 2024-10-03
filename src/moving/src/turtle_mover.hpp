#ifndef TURTLE_MOVER_HPP
#define TURTLE_MOVER_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>

class TurtleMover {
public:
    TurtleMover(double linear_speed, double tolerance)
        : linear_speed_(linear_speed), tolerance_(tolerance) {}

    geometry_msgs::msg::Twist calculate_movement(const turtlesim::msg::Pose& current_pose, const std::tuple<double, double>& target) {
        geometry_msgs::msg::Twist move_cmd;
        double distance = get_distance(current_pose, target);

        if (distance < tolerance_) {
            move_cmd.linear.x = 0;
            move_cmd.angular.z = 0;
        } else {
            move_cmd.linear.x = linear_speed_;
            move_cmd.angular.z = get_angle_diff(current_pose, target) * 100;
        }
        return move_cmd;
    }

private:
    double linear_speed_;
    double tolerance_;

    double get_angle_diff(const turtlesim::msg::Pose& current_pose, const std::tuple<double, double>& target) const {
        double angle_to_target = std::atan2(
            std::get<1>(target) - current_pose.y,
            std::get<0>(target) - current_pose.x
        );
        return normalize_angle(angle_to_target - current_pose.theta);
    }

    double get_distance(const turtlesim::msg::Pose& current_pose, const std::tuple<double, double>& target) const {
        return std::hypot(
            current_pose.x - std::get<0>(target),
            current_pose.y - std::get<1>(target)
        );
    }

    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

#endif // TURTLE_MOVER_HPP
