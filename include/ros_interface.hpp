#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "orchestrator.hpp"
#include "types.hpp"

namespace tv {

/// ROS2 node that wraps the full torque-vectoring pipeline.
///
/// Subscriptions:
///   ~/imu/data              sensor_msgs/Imu               — yaw_rate, ax, ay
///   ~/steering_command      std_msgs/Float64MultiArray[3]  — [steering_angle (rad), vx_ref (m/s),
///   r_ref (rad/s)]
///
/// Publication:
///   ~/wheel_torques         std_msgs/Float64MultiArray[4]  — [T_FL, T_FR, T_RL, T_RR] (Nm)
///
/// NOTE: VehicleState fields vx and vy (indices 0 and 1) are initialised to zero and
///       never updated by this node. Add an odometry subscriber if the controllers
///       require actual body velocities.
///
/// The pipeline is triggered on every IMU message. Steering updates are applied
/// immediately on receipt and will take effect on the next IMU tick.
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
class ROS : public rclcpp::Node {
   public:
    explicit ROS(Orchestrator<F, Y, A> orchestrator,
                 const std::string& node_name = "torque_vectoring")
        : Node(node_name), orchestrator_(std::move(orchestrator)) {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(std::move(msg)); });

        steering_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "steering_command", 10, [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                steeringCallback(std::move(msg));
            });

        torque_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("wheel_torques", 10);
    }

   private:
    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) {
        state_.yaw_rate_ = msg->angular_velocity.z;  // yaw_rate  [rad/s]
        state_.ax_ = msg->linear_acceleration.x;     // ax        [m/s²]
        state_.ay_ = msg->linear_acceleration.y;     // ay        [m/s²]

        const WheelTorques torques = orchestrator_.run(state_, steering_cmd_);

        std_msgs::msg::Float64MultiArray out;
        out.data = {torques[0], torques[1], torques[2], torques[3]};
        torque_pub_->publish(out);
    }

    void steeringCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 3) {
            RCLCPP_WARN(get_logger(),
                        "steering_command: expected 3 elements [steering_angle, vx_ref, r_ref], "
                        "got %zu — ignoring",
                        msg->data.size());
            return;
        }
        steering_cmd_.steering_angle_ = msg->data[0];  // steering_angle [rad]
        steering_cmd_.vx_ref_ = msg->data[1];          // vx_ref         [m/s]
        steering_cmd_.r_ref_ = msg->data[2];           // r_ref          [rad/s]
        state_.steering_angle_ = msg->data[0];         // mirror into VehicleState
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr steering_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;

    Orchestrator<F, Y, A> orchestrator_;
    VehicleState state_{};            // vx_, vy_, yaw_rate_, steering_angle_, ax_, ay_
    SteeringCommand steering_cmd_{};  // steering_angle_, vx_ref_, r_ref_
};

// CTAD deduction guides
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
ROS(Orchestrator<F, Y, A>) -> ROS<F, Y, A>;

template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
ROS(Orchestrator<F, Y, A>, const std::string&) -> ROS<F, Y, A>;

}  // namespace tv
