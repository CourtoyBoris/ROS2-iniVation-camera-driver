/*
Autor: BLAYES Hugo

ROS2 Implementation of the EROS algorithm
https://arxiv.org/pdf/2205.07657
*/

#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include "event_multiplexer.cpp"
#include "message/msg/event_batch.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include "surface.cpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class EROS_ROS:public EventMultiplexer{
public:
	EROS_ROS(std::string name):EventMultiplexer("EROS_"+name,name),name(name){
		//if true we get a temp decrease when a pixel is not update
		this->declare_parameter<bool>("temp_decrease",false);
		get_parameter("temp_decrease",temp_decrease);
		
		//value of the temp decrease
		this->declare_parameter<int>("temp_value",50);
		get_parameter("temp_value",temp_value);
		
		//image output of the EROS Algorithm
		out_eros = this->create_publisher<sensor_msgs::msg::Image>("EROS_OUT_"+name,1000);
		
		//EROS use past image to create new image 
		//this variable save the last image of EROS
		image2 = cv::Mat::zeros(640,480, CV_8U);

		//dynamic initialisation
	 	init_fait = new bool(false);
	}

private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_eros;
	
	ev::EROS surf;
	bool* init_fait;
	bool temp_decrease;
	int temp_value = 50;
	string name;
	
	cv::Mat image2;
	
	int x = 0;
	int y = 0;
	
	//callback for event input
	void topic_callback(const message::msg::EventBatch::SharedPtr msg){
		//dynamic init
		if(!*(init_fait)){
			x = msg->resolution[0];
			y = msg->resolution[1];
			
			surf.init(x,y,51,20000);
			image2 = cv::Mat::zeros(y,x, CV_8U);
			
			*init_fait = true;
		}
		
		std::vector<cv::Point> points;
		
		cv_bridge::CvImage img_bridge;
          	sensor_msgs::msg::Image img_msg;
		
		//update of EROS
		for(const auto &event: msg->events){
			surf.update(event.x,event.y);
			points.push_back(cv::Point(event.x,event.y));
		}
		
		//temp decrease
		if(temp_decrease){
		
			for(int i=0;i<y;i++){
				for(int j=0;j<x;j++){
					auto pixel = image2.at<uchar>(i,j);
					if(pixel<temp_value)image2.at<uchar>(i,j)=0;
					else image2.at<uchar>(i,j) = pixel-temp_value;
				}
			}
		
			cv::Mat image = surf.getSurface();
		
			for(auto &point:points){
				image2.at<uchar>(point.y,point.x) = image.at<uchar>(point.y,point.x); 
			}
			img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, image2);
		}else{
			img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, surf.getSurface());
		}

		//publish
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
	std::vector<rclcpp::Node::SharedPtr> eros_node;
	
	for(size_t i=0; i < camera_name.size();i++){
		auto node = std::make_shared<EROS_ROS>(camera_name[i]);
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
