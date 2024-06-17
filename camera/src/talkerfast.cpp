/*
Author : Courtoy Boris

ROS2 driver developed to manage the event-based cameras from iniVation
This node is used to read the data from the different streams and
to publish them to the corresponding topics
The number of events processed is capped by the max_events variable
to have a smooth run on the Raspberry Pi of the TurtleBot
Relies on the dv-processing library 
https://dv-processing.inivation.com/rel_1_7/index.html#
*/

#include <cstdio>
#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include "rclcpp/rclcpp.hpp"
#include "message/msg/event_batch.hpp"
#include "message/msg/trigger_batch.hpp"
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <boost/array.hpp>
#include <sys/time.h>
#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

// Set this flag to true to add an event visualizer within this node
// Do not forget to select the resolution adapted to the model used
bool display_events = false;

// Set this flag to true to add a frame visualizer within this node
bool display_frames = false;

// DAVIS
//cv::Size resolution = {346,260};

// DVXplorer / Mini
cv::Size resolution = {640,480};
dv::EventStore events;
dv::visualization::EventVisualizer visualizer(resolution);
dv::EventStreamSlicer slicer;


// Class managing the camera node capturing the different data streams
class Camera : public rclcpp::Node
{
public:
  Camera() : Node("camera")
  { 
    std::cout << "Camera node started" << std::endl;
    // Publish to the "events", "frames", "imu_camera" et "triggers"
    event_pub = this->create_publisher<message::msg::EventBatch>("events",1000);
    image_pub = this->create_publisher<sensor_msgs::msg::Image>("frames",1000);
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_camera",1000);
    trigger_pub = this->create_publisher<message::msg::TriggerBatch>("triggers",1000);
    
    
    auto const resolution = capture.getEventResolution();
    if (resolution.has_value()){
      x = resolution.value().width;
      y = resolution.value().height;
    }
    else 
      exit(-1);
    
    std::cout << "Camera found : " <<  capture.getCameraName().c_str() << std::endl;
    

    // Display all the possible streams
    if(capture.isEventStreamAvailable())
      std::cout << "Event stream available" << std::endl;
    if(capture.isFrameStreamAvailable())
      std::cout << "Frame stream available" << std::endl;
    if(capture.isImuStreamAvailable())
      std::cout << "Imu stream available" << std::endl;
    if(capture.isTriggerStreamAvailable())
      std::cout << "Trigger stream available" << std::endl;
    
    // Set the loop rate of the callback function to 5ms ~ 20O iterations/sec
    loop_rate = this->create_wall_timer(
      5ms, std::bind(&Camera::timer_callback, this));

    
    
    if(display_events){
      // Color used by the visualizer    
      visualizer.setBackgroundColor(dv::visualization::colors::white);
      visualizer.setPositiveColor(dv::visualization::colors::iniBlue);
      visualizer.setNegativeColor(dv::visualization::colors::darkGrey);

      slicer.doEveryTimeInterval(33ms, [&visualizer](const dv::EventStore &events) {
          
          // Generate an image based on the events accumulated during 33ms
          cv::Mat image = visualizer.generateImage(events);
          
          // Display the image
          cv::imshow("Events", image);
          cv::waitKey(2);

      });
    }
    
  }

private:
  // Retrieve data from the different streams of the camera every 5 ms
  // and publish them to the corresponding topics
  void timer_callback(){
    // Check if camera is still running
    if(capture.isRunning()){
      
      // Check the different data stream available
      if(capture.isEventStreamAvailable()){
        // Check if new EventStore available
        if (const auto events = capture.getNextEventBatch(); events.has_value()) {
          
          // Create the EventBatch message
          auto msg = message::msg::EventBatch();

          // Convert the EventStore into an EventBatch ROS message
          int size = events->size();
          auto const coordinates = events->coordinates();
          auto const ts = events->timestamps();
          auto const polarities = events->polarities();
          

          int max_events = 5000;
          int i;
          for(i=0; i < size && i<max_events; i++){
            msg.x.push_back(coordinates(i,0));
            msg.y.push_back(coordinates(i,1));
            msg.ts.push_back(ts(i));
            msg.polarity.push_back(polarities(i));
          }
          msg.resolution[0] = x;
          msg.resolution[1] = y;
          
          msg.size = i;

          // Publish the EventBatch to the "events" topic
          event_pub->publish(msg);

          if(display_events){
            slicer.accept(*events);
          }
        }  
      }
      
      if(capture.isFrameStreamAvailable()){
        // Check if a new frame is available
        if (const auto frame = capture.getNextFrame(); frame.has_value()) {
          // Convert the opencv frame into a ROS image message
          cv_bridge::CvImage img_bridge;
          sensor_msgs::msg::Image img_msg;
          img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, frame->image);
          img_bridge.toImageMsg(img_msg);

          // Publish the image message to the "frame" topic
          image_pub->publish(img_msg);
          
          // Uncomment to display the image in this node
          if(display_frames){
            cv::imshow("Image",frame->image);
            cv::waitKey(2);
          }
        } 
      }
      
      if(capture.isImuStreamAvailable()){
        // Check if a new imuBatch is available
        if (const auto imuBatch = capture.getNextImuBatch(); imuBatch.has_value() && !imuBatch->empty()) {
          int size = imuBatch->size();
          for(int i=0; i < size; i++){
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.orientation.x = 0;
            imu_msg.orientation.y = 0;
            imu_msg.orientation.z = 0;
            imu_msg.orientation.w = 0;

            const auto angular = imuBatch->at(i).getAngularVelocities();
            imu_msg.angular_velocity.x = angular[0];
            imu_msg.angular_velocity.y = angular[1];
            imu_msg.angular_velocity.z = angular[2];

            const auto linear = imuBatch->at(i).getAccelerations();
            imu_msg.linear_acceleration.x = linear[0];
            imu_msg.linear_acceleration.y = linear[1];
            imu_msg.linear_acceleration.z = linear[2];

            std::array<double, 9> tmp = {0};
            
            imu_msg.orientation_covariance = tmp;
            imu_msg.angular_velocity_covariance = tmp;
            imu_msg.linear_acceleration_covariance = tmp;
            imu_msg.orientation_covariance[0] = -1;
            imu_msg.angular_velocity_covariance[0] = -1;
            imu_msg.linear_acceleration_covariance[0] = -1;
            imu_pub->publish(imu_msg);       
            
          }
        }
      }

      if(capture.isTriggerStreamAvailable()){
        // Check if a new trigger
        if (const auto triggersBatch = capture.getNextTriggerBatch(); triggersBatch.has_value() && !triggersBatch->empty()) {
          // Create and fill a new triggerBatch message
          auto msg = message::msg::TriggerBatch();
          int size = triggersBatch->size();
          for(int i=0; i < size;i++){
            auto tmp = triggersBatch->at(i);
            msg.timestamps.push_back(tmp.timestamp);
            msg.triggers.push_back((int) tmp.type);
          }
          // Publish the triggerBatch message to the 
          trigger_pub->publish(msg);
        }
      } 
    }
    
    else{
      exit(0);
    }
  };
  // CameraCapture object representing the camera being read
  // Discover any iniVation device
  dv::io::CameraCapture capture;
  
  // Resolution of the camera
  int x;
  int y;

  // Publishers corresponding to the different data streams
  rclcpp::Publisher<message::msg::EventBatch>::SharedPtr event_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<message::msg::TriggerBatch>::SharedPtr trigger_pub;
  
  // Loop timer
  rclcpp::TimerBase::SharedPtr loop_rate;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
