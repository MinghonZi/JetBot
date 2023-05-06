#include <chrono>
#include <memory>
#include <numbers>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <icm20600.hh>

using namespace std::chrono_literals;


class ICM20600Publisher : public rclcpp::Node {
public:
  ICM20600Publisher()
  : Node("icm20600"), icm20600(0, true) {
    icm20600.set_power_mode(ICM20600::SIX_AXIS_LOW_NOISE);
    icm20600.set_accel_dlpf(ICM20600::ACCEL_RATE_1K_BW_420);
    icm20600.set_gyro_dlpf(ICM20600::GYRO_RATE_1K_BW_176);

    icm20600.set_accel_scale(ICM20600::FS_4G);
    icm20600.set_gyro_scale(ICM20600::FS_500_DPS);

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(  // 200 Hz -> 5 ms
      5ms, std::bind(&ICM20600Publisher::publish, this));
  }

private:
  void
  publish() {
    auto linear_accel = icm20600.accel_rd();
    auto angular_vel = icm20600.gyro_rd();

    msg.header.frame_id = "imu_link";
    msg.header.stamp = this->now();

    msg.linear_acceleration.x = linear_accel[0] * 9.80665;
    msg.linear_acceleration.y = linear_accel[1] * 9.80665;
    msg.linear_acceleration.z = linear_accel[2] * 9.80665;
    msg.angular_velocity.x = angular_vel[0] * std::numbers::pi / 180;
    msg.angular_velocity.y = angular_vel[1] * std::numbers::pi / 180;
    msg.angular_velocity.z = angular_vel[2] * std::numbers::pi / 180;

    // A zero covariance matrix will be interpreted as "covariance unknown"
    msg.linear_acceleration_covariance[0] = 0;
    msg.linear_acceleration_covariance[4] = 0;
    msg.linear_acceleration_covariance[8] = 0;
    msg.angular_velocity_covariance[0] = 0;
    msg.angular_velocity_covariance[4] = 0;
    msg.angular_velocity_covariance[8] = 0;
    // ICM20600 does not produce an orientation estimate
    msg.orientation_covariance[0] = -1;

    publisher_->publish(msg);
  }

  ICM20600 icm20600;
  sensor_msgs::msg::Imu msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};


auto main(int argc, char * argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICM20600Publisher>());
  rclcpp::shutdown();
}
