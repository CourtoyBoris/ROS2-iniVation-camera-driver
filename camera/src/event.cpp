/*
Author : Courtoy Boris

ROS2 driver developed to manage the event-based cameras from iniVation
This node reads the events from the "events" topic and generate a
frame representing them
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
#include <iostream>
#include <fstream>
#include <sys/time.h>

// Uncomment the resolution of the corresponding camera

// DAVIS
//cv::Size resolution = {346,260};

// DVXplorer / Mini
cv::Size resolution = {640,480};

// Initialize slicer and visualizer
dv::EventStore events;
dv::visualization::EventVisualizer visualizer(resolution);
dv::EventStreamSlicer slicer;

using std::placeholders::_1;
using namespace std::chrono_literals;

// Class managing the event visualizer node
class EventVisualizer : public rclcpp::Node
{
public:
  EventVisualizer()
  : Node("event_visualizer")
  {
    // Subscribe to the "events" topic
    event_sub = this->create_subscription<message::msg::EventBatch>(
      "events", 1000, std::bind(&EventVisualizer::topic_callback, this, _1));
    
    // Color used by the visualizer
    visualizer.setBackgroundColor(dv::visualization::colors::white);
    visualizer.setPositiveColor(dv::visualization::colors::iniBlue);
    visualizer.setNegativeColor(dv::visualization::colors::darkGrey);
    
    // Callback function used to slice the events by batch of 33ms
    slicer.doEveryTimeInterval(33ms, [&visualizer](const dv::EventStore &events) {
        
        // Generate an image based on the events accumulated during 33ms
        cv::Mat image = visualizer.generateImage(events);
        
        // Display the image
        cv::imshow("Received_events", image);
        cv::waitKey(2);
    });
    
  }


private:
  // Callback function called each time the node is reading a message from events
  void topic_callback(const message::msg::EventBatch::SharedPtr msg) 
  {
    // Retrieve data from the message
    dv::EventStore store;
    std::vector<short int> x = msg->x;
    std::vector<short int> y = msg->y;
    std::vector<long int> ts = msg->ts;
    std::vector<short int> polarity = msg->polarity;
    int size = msg->size;
    // Build a EventStore with those events and give it to the slicer
    for(int i=0; i < size; i++){
      store.emplace_back(ts[i],x[i],y[i],polarity[i]);
    }

    slicer.accept(store);
    };
    
  rclcpp::Subscription<message::msg::EventBatch>::SharedPtr event_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventVisualizer>());
  rclcpp::shutdown();
  return 0;
}
