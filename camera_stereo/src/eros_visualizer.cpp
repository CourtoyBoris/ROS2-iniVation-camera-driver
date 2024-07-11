/*
Autor: BLAYES Hugo

ROS2 visualizer for all Image algorithm and EROS algorithm
*/

#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include "visualizer.cpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <string>

using std::placeholders::_1;

class ErosVisualizer: public rclcpp::Node{
public:
	ErosVisualizer(std::string name):Node("Image_Visualizer_"+name),name(name){
		std::function<void(const sensor_msgs::msg::Image::SharedPtr)> callback = std::bind(&ErosVisualizer::topic_callback,this,_1);
	
		//name of the image input
		this->declare_parameter<std::string>("topic_name_in_image_visu","EROS_OUT");
		get_parameter("topic_name_in_image_visu",topic_name);
	
		visu = this->create_subscription<sensor_msgs::msg::Image>(topic_name+"_"+name,1000,callback);
		
		//bool variable for dynamic dimension of the camera 
		init_fait = new bool(false);	
	}
private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr visu;
	Visualizer visualizer;
	std::string name;
	std::string topic_name;
	bool* init_fait;
	
	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg){
		if(!*(init_fait)){
			int x_tmp = msg->height;
			int y_tmp = msg->width;
			
			visualizer.changeResolution(x_tmp,y_tmp);
			visualizer.changeName(topic_name+"_"+name);
			
			*init_fait = true;
		}
		
		//convert and send the image to the visualizer
		cv_bridge::CvImagePtr img_bridge;
		img_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		
		cv::Mat img = img_bridge->image;
		
		visualizer.addMatrice(img);	
	
	}

};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char* argv[]){
	//this part allows you to launch a node for each camera connected to the system
	rclcpp::init(argc,argv);
	
	std::vector<std::string> camera_name = dv::io::discoverDevices();
	std::vector<rclcpp::Node::SharedPtr> eros_node;
	
	for(size_t i=0; i<camera_name.size();i++){
		auto node = std::make_shared<ErosVisualizer>(camera_name[i]);
		eros_node.push_back(node);
	}
	
	std::vector<std::thread> threads;
	
	for(auto &node: eros_node){
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
