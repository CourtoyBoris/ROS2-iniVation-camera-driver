/*
Autor: BLAYES Hugo

ROS2 Accumulator for event camera 
more a pixel are present in a event batch, more the pixel are white.
After taking into account all the pixels we carry out a histogram equalization. 
*/
#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include "event_multiplexer.cpp"
#include "message/msg/event_batch.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Accumulator:public EventMultiplexer{
public:
	Accumulator(std::string name):EventMultiplexer("ACCUMULATOR_"+name,name),name(name){
		//if true we get a temp decrease when a pixel is not update
		this->declare_parameter<bool>("temp_decrease",false);
		get_parameter("temp_decrease",temp_decrease);
		
		//value of the temp decrease
		this->declare_parameter<int>("temp_value",50);
		get_parameter("temp_value",temp_value);
		
		//this publisher publish the out image of the accumulator 
		out_eros = this->create_publisher<sensor_msgs::msg::Image>("ACCUMULATOR_OUT_"+name,1000);
		
		//dynamic dimension of the camera 
	 	init_fait = new bool(false);
	}

private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_eros;

	bool* init_fait;
	bool temp_decrease;
	int temp_value = 50;
	string name;
	
	cv::Mat image;
	
	int x;
	int y;
	
	//callback of the event multiplexer
	void topic_callback(const message::msg::EventBatch::SharedPtr msg){
		//dynamic init
		if(!*(init_fait)){
			x = msg->resolution[0];
			y = msg->resolution[1];
			
			*init_fait = true;
		}
		
		//accumulator declaration
		image = cv::Mat::zeros(y,x, CV_8U);
		
		for(auto &event: msg->events){
			image.at<uchar>(event.y,event.x) += 1;
		}
		
		cv::equalizeHist(image,image);
		
		//transform to CvImage
		cv_bridge::CvImage img_bridge;
        sensor_msgs::msg::Image img_msg;
		img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, image);
        img_bridge.toImageMsg(img_msg);
        out_eros->publish(img_msg);
	}
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char** argv){
	//this part allows you to launch a node for each camera connected to the system
	rclcpp::init(argc, argv);
	
	std::vector<std::string> camera_name = dv::io::discoverDevices();
	std::vector<rclcpp::Node::SharedPtr> accumulator_node;
	
	for(size_t i=0; i < camera_name.size();i++){
		auto node = std::make_shared<Accumulator>(camera_name[i]);
		accumulator_node.push_back(node);
	}
	
	std::vector<std::thread> threads;
	
	for(auto &node: accumulator_node){
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
