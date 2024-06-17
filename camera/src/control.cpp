/*
Author : Courtoy Boris

ROS2 driving mechanism used to control a TurtleBot3 based on
the events retrieved by the camera
The resolution of the camera is divided in 9 bins and the
motion order sent to the robot depends on which bins
contains the highest number of events
Relies on the dv-processing library 
https://dv-processing.inivation.com/rel_1_7/index.html#
*/

#include <cstdio>
#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include "rclcpp/rclcpp.hpp"
#include "message/msg/event_batch.hpp"
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <boost/array.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Class managing the control node reading the event stream and controlling the robot
class Test : public rclcpp::Node
{
public:
  Test() : Node("control")
  {
    // Subscribe to the events topic and publish to the cmd_vel topic
    event_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1000);
    event_sub = this->create_subscription<message::msg::EventBatch>(
      "events", 1000, std::bind(&Test::topic_callback, this, _1));
    
  }

private:
  void topic_callback(const message::msg::EventBatch::SharedPtr msg) const{
    dv::EventStore store;
    std::vector<short int> x = msg->x;
    std::vector<short int> y = msg->y;
    std::vector<long int> ts = msg->ts;
    std::vector<short int> polarity = msg->polarity;
    
    std::vector<int> cpt(9,0); 

    int size = msg->size;

    // Count the number of events in each bin from the retrieved event batch
    for(int i=0; i < size; i++){
      if(x[i] <= 640/3){
        if(y[i] <= 480/3)
          cpt[0]++;
        else if(y[i] <= 2*(480/3))
          cpt[3]++;
        else if(y[i] <= 480)
          cpt[6]++;
      }
      else if(x[i] <= 2*(640/3)){
        if(y[i] <= 480/3)
          cpt[1]++;
        else if(y[i] <= 2*(480/3))
          cpt[4]++;
        else if(y[i] <= 480)
          cpt[7]++;
      }
      else if(x[i] <= 640){
        if(y[i] <= 480/3)
          cpt[2]++;
        else if(y[i] <= 2*(480/3))
          cpt[5]++;
        else if(y[i] <= 480)
          cpt[8]++;
      }
    }

    int max = 4;

    // Only trigger the action if the number of events in the max bin exceeds a threshold
    for(int i=0; i < (int) cpt.size(); i++){
      if(cpt[i] > cpt[max])
        max = i;
    }

    if(cpt[max] < 800)
      max = 4;
    auto message = geometry_msgs::msg::Twist();

    message.linear.y = 0;
    message.linear.z = 0;

    message.angular.x = 0;
    message.angular.y = 0;

    // Trigger the action corresponding to the max bin

    switch(max){
      case 0:
        message.linear.x = 0.1;

        message.angular.z = 0.3;
        std::cout <<  "forward + left"  << std::endl;
        break;
      case 1:
        message.linear.x = 0.1;
        
        message.angular.z = 0;
        std::cout <<  "forward"  << std::endl;
        break;
      case 2:
        message.linear.x = 0.1;

        message.angular.z = -0.3;
        std::cout <<  "forward + right"  << std::endl;
        break;
      case 3:
        message.linear.x = 0;

        message.angular.z = 0.3;
        std::cout <<  "left"  << std::endl;
        break;
      case 4:
        message.linear.x = 0;

        message.angular.z = 0;
        std::cout <<  "stay"  << std::endl;
        break;
      case 5:
        message.linear.x = 0;

        message.angular.z = -0.3;
        std::cout <<  "right"  << std::endl;
        break;
      case 6:
        message.linear.x = -0.1;

        message.angular.z = 0.3;
        std::cout <<   "backward + left" << std::endl;
        break;
      case 7:
        message.linear.x = -0.1;

        message.angular.z = 0;
        std::cout <<  "backward"   << std::endl;
        break;
      case 8:
        message.linear.x = -0.1;

        message.angular.z = -0.3;
        std::cout <<  "bacwkard + right"   << std::endl;
        break;
    }

    event_pub->publish(message);
    }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr event_pub;
  rclcpp::Subscription<message::msg::EventBatch>::SharedPtr event_sub;

  rclcpp::TimerBase::SharedPtr loop_rate;
};


int main(int argc, char ** argv)
{
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Test>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
