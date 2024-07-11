#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include "event_multiplexer.cpp"
#include "message/msg/event_batch.hpp"
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"
#include "surface.cpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WINNER:public EventMultiplexer{
public:
	WINNER(std::string name):EventMultiplexer("WINNER_"+name,name),name(name){
		out_winner = this->create_publisher<sensor_msgs::msg::Image>("WINNER_OUT_"+name,1000);
		
	 	init_fait = new bool(false);
	}

private:
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr out_winner;

	bool* init_fait;
	string name;
	
	cv::Mat image;
	
	int x = 0;
	int y = 0;
	
	void topic_callback(const message::msg::EventBatch::SharedPtr msg){
		if(!*(init_fait)){
			x = msg->resolution[0];
			y = msg->resolution[1];
			
			*init_fait = true;
		}
		
		image = cv::Mat::zeros(y+1,x+1, CV_8U);
		
		for(const auto &event: msg->events){
			int x_tmp = event.x;
			int y_tmp = event.y;
			if(image.at<uchar>(y_tmp  ,x_tmp)!=0)image.at<uchar>(y_tmp  ,x_tmp) -= 1;			
			if(image.at<uchar>(y_tmp+1,x_tmp)!=0)image.at<uchar>(y_tmp+1,x_tmp) -= 1;
			if(image.at<uchar>(y_tmp+2,x_tmp)!=0)image.at<uchar>(y_tmp+2,x_tmp) -= 1;
			
			if(image.at<uchar>(y_tmp  ,x_tmp+1)!=0)image.at<uchar>(y_tmp,x_tmp+1) -= 1;
			image.at<uchar>(y_tmp+1,x_tmp+1) = 100;
			if(image.at<uchar>(y_tmp+2,x_tmp+1)!=0)image.at<uchar>(y_tmp+2,x_tmp+1) -= 1;
			
			if(image.at<uchar>(y_tmp,x_tmp+2)!=0)image.at<uchar>(y_tmp,x_tmp+2) -= 1;
			if(image.at<uchar>(y_tmp+1,x_tmp+2)!=0)image.at<uchar>(y_tmp+1,x_tmp+2) -= 1;
			if(image.at<uchar>(y_tmp+2,x_tmp+2)!=0)image.at<uchar>(y_tmp+2,x_tmp+2) -= 1;
			
		}
	
		cv::equalizeHist(image,image);
	
		cv_bridge::CvImage img_bridge;
          	sensor_msgs::msg::Image img_msg;
		img_bridge = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, image);
          	img_bridge.toImageMsg(img_msg);
          	out_winner->publish(img_msg);
	}
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	
	std::vector<std::string> camera_name = dv::io::discoverDevices();
	std::vector<rclcpp::Node::SharedPtr> winner_node;
	
	for(size_t i=0; i < camera_name.size();i++){
		auto node = std::make_shared<WINNER>(camera_name[i]);
		winner_node.push_back(node);
	}
	
	std::vector<std::thread> threads;
	
	for(auto &node: winner_node){
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
