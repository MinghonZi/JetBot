// TODO: Use image transport instead of transmitting raw images
// #include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp> // Include whole OpenCV; compilation time isn't a concern currently.

#include <yolov8.hh>

using std::placeholders::_1;


class DuckDetector : public rclcpp::Node {
public:
  DuckDetector()
  // FIXME: Hard-coded engine file path
  : Node("duck_detector"), model("/ws/weights/duck.engine") {
    model.make_pipe(true);

    publisher_ = this->create_publisher<std_msgs::msg::Bool>("/duck_detected", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/csi_cam", 10, std::bind(&DuckDetector::detect, this, _1));
  }

private:
  void
  detect(const sensor_msgs::msg::Image::SharedPtr msg) {
		model.copy_from_Mat(cv_bridge::toCvShare(msg, "bgr8")->image, size);
    model.infer();
    model.postprocess(objs);

    if (objs.empty()) {
      if (detected.data == true) {
        detected.set__data(false);
        publisher_->publish(detected);
      }
      return;
    }

    // objs only contains one obj whose label is 0 stands for rubber duck
    auto duck = objs[0];

    if (duck.prob >= 0.7 && detected.data == false) {
      detected.set__data(true);
      publisher_->publish(detected);
      return;
    }

    if (duck.prob < 0.7 && detected.data == true) {
      detected.set__data(false);
      publisher_->publish(detected);
      return;
    }
  }

  YOLOv8 model;
  cv::Size size = cv::Size{640, 640};
  std::vector<Object> objs;

  std_msgs::msg::Bool detected;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};


auto main(int argc, char * argv[]) -> int {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DuckDetector>());
  rclcpp::shutdown();
}
