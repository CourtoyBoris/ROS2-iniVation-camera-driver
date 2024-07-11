/*
Autor: BLAYES Hugo

This executable is the same as the talker_duo but the eros algorithm
are directly plug into the file to optimise for small configuration like the turtlebot
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
#include <string>
#include "message/msg/num.hpp"
#include "surface.cpp"
#include <fstream>

using namespace std::chrono_literals;


class Camera : public rclcpp::Node{
public:
  Camera(std::string cam) : Node("camera_"+cam)
  {
    cameraCapturePtr = std::make_unique<dv::io::CameraCapture>(cam);
   
    this->declare_parameter<int>("number_of_topics", 1); 
    this->declare_parameter<int>("cap_of_event",5000);
    this->declare_parameter<bool>("frame_activate",false);
    this->declare_parameter<bool>("imu_activate",false);
    this->declare_parameter<bool>("trigger_activate",false);
    this->declare_parameter<std::string>("event_topic_name","events");
    get_parameter("number_of_topics",NbTopics);
    get_parameter("cap_of_event",NbEvents);
    get_parameter("frame_activate",FrameActivate);
    get_parameter("imu_activate",ImuActivate);
    get_parameter("trigger_activate",TriggerActivate);
    get_parameter("event_topic_name",EventTopicName);
    
    out_eros = this->create_publisher<sensor_msgs::msg::Image>("EROS_OUT_"+cam,1000);
  
    if(FrameActivate)image_pub = this->create_publisher<sensor_msgs::msg::Image>("frames_"+cam,1000);
    if(ImuActivate)imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_"+cam,1000);
    if(TriggerActivate)trigger_pub = this->create_publisher<message::msg::Num>("trigger_"+cam,100);
    auto const resolution = cameraCapturePtr->getEventResolution();
    if (resolution.has_value()){
      x = resolution.value().width;
      y = resolution.value().height;
      std::cout << x << std::endl;
      std::cout << y << std::endl;
    }
    else 
      exit(-1);
    surf.init(x,y,5,0.5);
    
    
    loop_rate = this->create_wall_timer(
      5ms, std::bind(&Camera::timer_callback, this));
    
    cameraCapturePtr->setDVSBiasSensitivity(dv::io::CameraCapture::BiasSensitivity::VeryLow);
  }

private:
  void timer_callback(){
    if(cameraCapturePtr->isRunning()){
      if(cameraCapturePtr->isEventStreamAvailable()){
        if (const auto events = cameraCapturePtr->getNextEventBatch(); events.has_value()) {
		int i = 0;
		for(const auto &event: events->slice(0,events->size())){
			
			/*if(event.polarity()==0)surf.update(event.x(),event.y(),event.timestamp(),-1);		
			else*/ surf.update(event.x(),event.y(),event.timestamp(),event.polarity());  		
     		}
     		
     		cv_bridge::CvImage img_bridge;
          	sensor_msgs::msg::Image img_msg;
          	img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, surf.getSurface());
          		
          	img_bridge.toImageMsg(img_msg);
          	out_eros->publish(img_msg);  
    	}
    }

    if(ImuActivate){
      if(cameraCapturePtr->isImuStreamAvailable()){
        if (const auto imuBatch = cameraCapturePtr->getNextImuBatch(); imuBatch.has_value() && !imuBatch->empty()) {
          for(size_t i=0; i < imuBatch->size(); i++){
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
            imu_pub->publish(imu_msg);       
          }
        }
      }
     }
      
     if(TriggerActivate){
     	if(cameraCapturePtr->isTriggerStreamAvailable()){
     	  if(const auto triggersBatch = cameraCapturePtr->getNextTriggerBatch();triggersBatch.has_value() && !triggersBatch->empty()){
     	    auto msg = message::msg::Num();
     	    for(size_t i=0; i<triggersBatch->size();i++){
     	      auto tmp = triggersBatch->at(i);
     	      msg.timestamps.push_back(tmp.timestamp);
     	      msg.triggers.push_back((int) tmp.type);
     	    }
     	    trigger_pub->publish(msg);
     	  }
     	}
     }  
      
     if(!FrameActivate)return;
      
     if(cameraCapturePtr->isFrameStreamAvailable()){
       if (const auto frame = cameraCapturePtr->getNextFrame(); frame.has_value()) {
         cv_bridge::CvImage img_bridge;
         sensor_msgs::msg::Image img_msg;
         img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, frame->image);
         img_bridge.toImageMsg(img_msg);

         image_pub->publish(img_msg);
       }

       cv::waitKey(1);
     }
    }
    else{
      
      exit(0);
    }
  }
  
  
  std::unique_ptr<dv::io::CameraCapture> cameraCapturePtr;
  
  int x;
  int y;
  int NbTopics=0;
  int NbEvents=5000;
  bool FrameActivate=false;
  bool ImuActivate=false;
  bool TriggerActivate=false;
  std::string EventTopicName;
  ev::EROS surf;
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<message::msg::Num>::SharedPtr trigger_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_eros;
  
  rclcpp::TimerBase::SharedPtr loop_rate;
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char ** argv){
    
    rclcpp::init(argc, argv);
    
    std::vector<std::string> camera_name = dv::io::discoverDevices();
    std::vector<rclcpp::Node::SharedPtr> camera_node;
    
    for(size_t i=0; i < camera_name.size(); i++){
    	auto node = std::make_shared<Camera>(camera_name[i]);
    	camera_node.push_back(node);
    }
    
    std::vector<std::thread> threads;
    
    for(auto &node: camera_node){
    	threads.push_back(std::thread(run_node,node));
    }
    
    for(auto &thread: threads){
    	if(thread.joinable()){
    		thread.join();
    	}
    }
    
    rclcpp::shutdown();

    return 0;
}
