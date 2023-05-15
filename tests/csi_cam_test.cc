#include <opencv2/opencv.hpp>

int main()
{
  cv::VideoCapture cap("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! nvvidconv flip-method=0 ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
  if (!cap.isOpened())
  {
    std::cout << "Failed to open camera." << std::endl;
    return EXIT_FAILURE;
  }

  cv::Mat img;
  if (!cap.read(img))
  {
    std::cout << "Capture read error." << std::endl;
    return EXIT_FAILURE;
  }
  cv::imwrite("test.png", img);

  cap.release();
}
