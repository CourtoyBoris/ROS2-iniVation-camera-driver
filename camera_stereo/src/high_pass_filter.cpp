/*
Autor: BLAYES Hugo

ROS2 High pass filter for event or image
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

class High_Pass_Filter:public EventMultiplexer{
public:
	High_Pass_Filter(std::string name):EventMultiplexer("high_pass_"+name,name),name(name){
		this->declare_parameter<int>("kernel_size",3);
		this->declare_parameter<std::string>("topic_name_in_high_pass","high_pass_in");
		get_parameter("kernel_size_high_pass",KernelSize);
		get_parameter("topic_name_in_high_pass",topic_name);
		
		
		out_high_pass = this->create_publisher<sensor_msgs::msg::Image>("HIGH_PASS_OUT_"+name,1000);
		std::function<void(const sensor_msgs::msg::Image::SharedPtr)> callback = std::bind(&High_Pass_Filter::image_callback,this,_1);
		image_sub = this->create_subscription<sensor_msgs::msg::Image>(topic_name+"_"+name,1000,callback);
		
		//Kernel declaration and definition
		if(KernelSize%2==0)KernelSize++;
		highPassKernel = cv::Mat::zeros(KernelSize,KernelSize,CV_32F);
		for(int i=0;i<KernelSize;i++){
			for(int j=0;j<KernelSize;j++){
				highPassKernel.at<float>(i,j) = -(KernelSize*KernelSize-1);
			}
		}
		
		highPassKernel.at<float>(KernelSize/2,KernelSize/2) = KernelSize*KernelSize-1;
		
		init_fait = new bool(false);
	}
private:
	std::string name;
	std::string topic_name;
	bool* init_fait;
	
	int x=0;
	int y=0;
	
	int KernelSize = 3;
	
	cv::Mat highPassKernel;
	
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_high_pass;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub; 

	//event callback
	void topic_callback(const message::msg::EventBatch::SharedPtr msg){
		if(!*init_fait){
			x = msg->resolution[0];
			y = msg->resolution[1];
			
			*init_fait = true;
		}
		
		//convert event into an image
		cv::Mat image = cv::Mat::zeros(x,y,CV_32F);
		
		for(const auto &event: msg->events){
			int polarity = event.polarity;
			if(polarity==1){
				image.at<float>(event.y,event.x) = 255;
			}else{
				image.at<float>(event.y,event.x) = 128;
			}
		}
		
		//applying the filter
		cv::filter2D(image,image,CV_32F, highPassKernel);
		image.convertTo(image,CV_8U);
		
		//equalize histogramm for better visualisation 
		cv::equalizeHist(image,image);
		
		//convert to ROS2 msg publish
		cv_bridge::CvImage img_bridge;
		sensor_msgs::msg::Image img_msg;
		img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::MONO8, image);
		img_bridge.toImageMsg(img_msg);
		out_high_pass->publish(img_msg);	
	}
	
	//image callback
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
		cv_bridge::CvImagePtr img_bridge;
		img_bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

		cv::Mat image = img_bridge->image;
		
		//applying the filter
		cv::filter2D(image,image,CV_32F, highPassKernel);
		image.convertTo(image,CV_8U);
		
		//equalize histogramm for better visualisation 
		cv::equalizeHist(image,image);
		
		//convert to ROS2 msg publish
		sensor_msgs::msg::Image img_msg;
		cv_bridge::CvImage img_bridge_b = cv_bridge::CvImage(std_msgs::msg::Header(),sensor_msgs::image_encodings::MONO8, image);
		img_bridge_b.toImageMsg(img_msg);
		out_high_pass->publish(img_msg);
	}
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char** argv){
	//this part allows you to launch a node for each camera connected to the system
	rclcpp::init(argc, argv);
	
	std::vector<std::string> camera_name = dv::io::discoverDevices();
	std::vector<rclcpp::Node::SharedPtr> high_node;
	
	for(size_t i=0; i<camera_name.size();i++){
		auto node = std::make_shared<High_Pass_Filter>(camera_name[i]);
		high_node.push_back(node);
	}
	
	std::vector<std::thread> threads;
	
	for(auto &node: high_node){
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
