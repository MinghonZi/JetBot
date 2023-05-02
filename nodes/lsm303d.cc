#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <lsm303d.hh>

using namespace std::chrono_literals;


class LSM303DPublisher : public rclcpp::Node {
public:
  LSM303DPublisher()
  : Node("lsm303d"), lsm303d(0, std::byte{0x1d}) {
    lsm303d.control1_wr(std::byte{0b0101'0'1'1'1});
    lsm303d.control2_wr(std::byte{0b00'000'0'0'0});
    lsm303d.control3_wr(std::byte{0b0'0'0'0'0'0'0'0});
    lsm303d.control4_wr(std::byte{0b0'0'0'0'0'0'0'0});
    lsm303d.control5_wr(std::byte{0b0'11'100'0'0});
    lsm303d.control6_wr(std::byte{0b0'00'00000});
    lsm303d.control7_wr(std::byte{0b00'0'0'0'0'00});

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(  // 100 Hz -> 10 ms
      10ms, std::bind(&LSM303DPublisher::publish, this));
  }

private:
  void
  publish() {
    auto acceleration = lsm303d.accelerometer_rd();

    msg.header.frame_id = "imu_link";
    msg.header.stamp = this->now();

    msg.linear_acceleration.x = acceleration[0];
    msg.linear_acceleration.y = acceleration[1];
    msg.linear_acceleration.z = acceleration[2];

    // A zero covariance matrix will be interpreted as "covariance unknown"
    msg.linear_acceleration_covariance[0] = 0;
    msg.linear_acceleration_covariance[4] = 0;
    msg.linear_acceleration_covariance[8] = 0;
    // LSM303D does not produce an orientation and angular velocity estimate
    msg.orientation_covariance[0] = -1;
    msg.angular_velocity_covariance[0] = -1;

    publisher_->publish(msg);
  }

  LSM303D lsm303d;
  sensor_msgs::msg::Imu msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};


auto main(int argc, char * argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LSM303DPublisher>());
  rclcpp::shutdown();
}
