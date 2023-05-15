/**
* Foxglove teleop panel settings
* - Publish rate: 5.00
* - Topic: /cmd_vel
* - Up   : linear-x   0.56
* - Down : linear-x  -0.56
* - Left : angular-z  8.00
* - Right: angular-z -8.00
* Refer to https://foxglove.dev/docs/studio/panels/teleop
* 
* Alternatives
* - ROS1: https://wiki.ros.org/diff_drive_controller
* - ROS2: https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
*/

#include <algorithm>
#include <chrono>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

// FIXME
inline constexpr auto MAX_RPM = 180; // Roughly measured
inline constexpr auto MAX_MAV = MAX_RPM * 2 * std::numbers::pi / 60; // Max motor angular velocity (rad/s)
inline constexpr auto MIN_MAV = -MAX_MAV; // Min motor angular velocity (rad/s)
inline constexpr auto WHEEL_RADIUS = 0.03; // m
inline constexpr auto WHEEL_SEPERATION = 0.1; // m

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace py = pybind11;

// TODO: https://stackoverflow.com/q/60927251/20015297
py::scoped_interpreter guard{};


class Controller : public rclcpp::Node {
public:
  Controller()
  : Node("controller") {
    // Period allowed between two successive velocity commands.
    // After this delay, a zero speed command will be sent to the wheels.
    cmd_vel_timeout_ = this->create_wall_timer(
    300ms, std::bind(&Controller::brake, this)); // FIXME: Expose the timeout time

    // FIXME: QoS profile
    cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Controller::cmd_vel, this, _1));
  }

private:
  void
  cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vel_timeout_->reset();

    // https://answers.ros.org/question/185427/make-sure-geometry_msgstwist-or-cmd_vel-units/
    auto 𝑣 = msg->linear.x; // Max 0.56 m/s (3 revolutions of the motor)
    auto 𝜔 = msg->angular.z; // rad/s
    auto 𝑟 = WHEEL_RADIUS;
    auto 𝑏 = WHEEL_SEPERATION;

    // https://en.wikipedia.org/wiki/Differential_wheeled_robot#Kinematics_of_Differential_Drive_Robots
    auto 𝜔ᴸ = (𝑣 - 𝜔 * 𝑏 / 2) / 𝑟;
    auto 𝜔ᴿ = (𝑣 + 𝜔 * 𝑏 / 2) / 𝑟;

    𝜔ᴸ = std::clamp(𝜔ᴸ, MIN_MAV, MAX_MAV);
    𝜔ᴿ = std::clamp(𝜔ᴿ, MIN_MAV, MAX_MAV);

    /**
     * FIXME: These are very basic motors, and have no built-inencoders, speed
     * control or positional feedback. Voltage goes in, rotation goes out!
     *
     * Normalise to [-1, 1] range
     * https://en.wikipedia.org/wiki/Normalization_(statistics)
     * https://en.wikipedia.org/wiki/Feature_scaling
     * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
     */
    lmotor.attr("throttle") = (𝜔ᴸ - MIN_MAV) * (1 - -1) / (MAX_MAV - MIN_MAV) + -1;
    rmotor.attr("throttle") = (𝜔ᴿ - MIN_MAV) * (1 - -1) / (MAX_MAV - MIN_MAV) + -1;
  }

  void
  brake() {
    lmotor.attr("throttle") = 0.0;
    rmotor.attr("throttle") = 0.0;
  }

  // https://adafruit.com/product/2927
  // https://adafruit.com/product/3777
  py::object MotorKit = py::module::import("adafruit_motorkit").attr("MotorKit");
  py::object kit = MotorKit();
  py::object lmotor = kit.attr("motor1");
  py::object rmotor = kit.attr("motor2");

  rclcpp::TimerBase::SharedPtr cmd_vel_timeout_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
};


auto main(int argc, char * argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
