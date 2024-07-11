/*
Autor: BLAYES Hugo

ROS2 Blur filter (low phase filter) for event based camera
*/

#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include "event_multiplexer.cpp"
#include "message/msg/event_batch.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>

using std::placeholders::_1;

class Blur_Filter:public EventMultiplexer{
public:
	Blur_Filter(std::string name):EventMultiplexer("BLUR_"+name,name),name(name){
		//parameter kernel size and topic name for in
		this->declare_parameter<int>("kernel_size_blur",3);
		this->declare_parameter<std::string>("topic_name_in_blur","blur_in");
		get_parameter("kernel_size_blur",KernelSize);
		get_parameter("topic_name_in_blur",topic_name);
		
		//publisher for the image output
		out_blur = this->create_publisher<sensor_msgs::msg::Image>("BLUR_OUT_"+name,1000);
		
		//subscriber for the image input
		std::function<void(const sensor_msgs::msg::Image::SharedPtr)> callback = std::bind(&Blur_Filter::image_callback,this,_1);
		image_sub = this->create_subscription<sensor_msgs::msg::Image>(topic_name+"_"+name,1000,callback);
		
		//bool for dynamic initialisation 
		init_fait = new bool(false);
	}
private:
	std::string name;
	std::string topic_name;
	bool* init_fait;
	
	int x=0;
	int y=0;
	
	int KernelSize = 3;
	
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_blur;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub; 

	//callback for the event input
	void topic_callback(const message::msg::EventBatch::SharedPtr msg){
		if(!*init_fait){
			x = msg->resolution[0];
			y = msg->resolution[1];
			
			*init_fait = true;
		}
		
		cv::Mat image = cv::Mat::zeros(x,y,CV_8U);
		
		//We convert events into a image
		for(const auto &event: msg->events){
			int polarity = event.polarity;
			if(polarity==1){
				image.at<unsigned char>(event.y,event.x) = 255;
			}else{
				image.at<unsigned char>(event.y,event.x) = 128;
			}
		}
		
		cv::blur(image,image,cv::Size(KernelSize,KernelSize));
		
		cv_bridge::CvImage img_bridge;
		sensor_msgs::msg::Image img_msg;
		img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::MONO8, image);
		img_bridge.toImageMsg(img_msg);
		out_blur->publish(img_msg);	
	}
	
	//callback for the image input
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
		cv_bridge::CvImagePtr img_bridge;
		img_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

		cv::Mat image = img_bridge->image;
		
		cv::blur(image,image,cv::Size(KernelSize,KernelSize));

		sensor_msgs::msg::Image img_msg;
		cv_bridge::CvImage img_bridge_b = cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::MONO8, image);
		img_bridge_b.toImageMsg(img_msg);
		out_blur->publish(img_msg);
	}
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char** argv){
	//this part allows you to launch a node for each camera connected to the system
	rclcpp::init(argc, argv);
	
	std::vector<std::string> camera_name = dv::io::discoverDevices();
	std::vector<rclcpp::Node::SharedPtr> blur_node;
	
	for(size_t i=0; i<camera_name.size();i++){
		auto node = std::make_shared<Blur_Filter>(camera_name[i]);
		blur_node.push_back(node);
	}
	
	std::vector<std::thread> threads;
	
	for(auto &node: blur_node){
		threads.push_back(std::thread(run_node, node));
	}
	
	for(auto &thread:threads){
		if(thread.joinable()){
			thread.join();
		}
	}
	
	rclcpp::shutdown();
	
	return 0;
}
