/*
Autor: BLAYES Hugo

Event visualizer
*/

#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include "message/msg/event_batch.hpp"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualizer.cpp"
#include <string>
#include "event_multiplexer.cpp"

#include <sys/time.h>
#include <iostream>
#include <fstream>

class EventVisualizer : public EventMultiplexer{
public:
  EventVisualizer(std::string name):EventMultiplexer("event_visualizer_"+name,name),name(name){
    //bool variable for dynamic dimension of the camera 
    init_fait = new bool(false);
  }

private:
  void topic_callback(const message::msg::EventBatch::SharedPtr msg){
    //dynamic init
    if(!*(init_fait)){
       int x_tmp = msg->resolution[0];
       int y_tmp = msg->resolution[1];
       
       visualizer.changeResolution(x_tmp,y_tmp);
       visualizer.changeName("EROS_"+name);
       
       *init_fait = true;
    }
    
    //add event one per one to the visualizer
    for(const auto &event: msg->events){
	     visualizer.addEvent(event.y,event.x,event.polarity);
    }
 }
  
  int temp = 0;
  string name;
  Visualizer visualizer;
  bool* init_fait;
  std::vector<int> compteur; 
  std::vector<int> nb_topic;
  
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char * argv[]){
  //this part allows you to launch a node for each camera connected to the system
  rclcpp::init(argc, argv);

  std::vector<std::string> camera_name = dv::io::discoverDevices();
  std::vector<rclcpp::Node::SharedPtr> event_node;
  
  for(size_t i=0; i< camera_name.size();i++){
  	std::cout << camera_name[i] << std::endl;
  	auto node = std::make_shared<EventVisualizer>(camera_name[i]);
  	event_node.push_back(node);
  }
  
  std::vector<std::thread> threads;
  
  for(auto &node: event_node){
  	threads.push_back(std::thread(run_node,node));
  }

  for(auto &thread:threads){
  	if(thread.joinable()){
  		thread.join();
  	}
  }
  
  rclcpp::shutdown();
  return 0;
}
