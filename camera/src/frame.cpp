/*
Author : Courtoy Boris

ROS2 driver developed to manage the event-based cameras from iniVation
This node reads the frames from the "frames" topic and display them
Relies on the dv-processing library 
https://dv-processing.inivation.com/rel_1_7/index.html#
*/

#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include "rclcpp/rclcpp.hpp"
#include "message/msg/event_batch.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp"


using std::placeholders::_1;

// Class managing the frame visualizer node
class FrameVisualizer : public rclcpp::Node
{
public:
  FrameVisualizer()
  : Node("frame_visualizer")
  {
    // Subscribe to the "frames" topic
    image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      "frames", 1000, std::bind(&FrameVisualizer::topic_callback, this, _1));
  }

private:
// Callback function called each time the node is reading a message from frames
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    // Convert the image ROS message into an opencv image using cv_bridge
    cv_bridge::CvImagePtr img_bridge;
    img_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv::namedWindow("Received image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Received image", img_bridge->image);
    cv::waitKey(2);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameVisualizer>());
  rclcpp::shutdown();
  return 0;
}
