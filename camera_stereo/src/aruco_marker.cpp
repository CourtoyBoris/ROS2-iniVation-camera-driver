/*
Autor: BLAYES Hugo

Draft of an Aruco marker detector with an event-based camera.

NOT TEST!!

*/

#include <dv-processing/io/camera_capture.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/objdetect/aruco_detector.hpp>

using std::placeholders::_1;

class Aruco_Detector:public Node{
public:
	Aruco_Detector(std::string name):Node("Aruco_Detector_"+name),name(name){
		std::function<void(const sensor_msgs::msg::Image::SharedPtr)> callback = std::bind(&Aruco_Detector::topic_callback, this, _1);
		image_sub = this->create_subscription<sensor_msgs::msg::Image>("Aruco_Detector_"+name,1000,callback);
	}
private:
	std::string name;
	
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
	
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::Dictionary dico = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::ArucoDetector detector(dictionnary, detectorParams);
	
	void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg){
		cv_bridge::CvImagePtr img_bridge;
		img_bridge = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
		
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2d>> markerCorners, rejectedCandidates;	
		
		detector.detectMarkers(img_bridge->image,markerCorners, markerIds, rejectedCandidates);
		
		for(int &id:markerIds){
			std::cout << id << std::endl;
		}
	}
};

void run_node(rclcpp::Node::SharedPtr node){
	rclcpp::spin(node);
}

int main(int argc, char** argv){
	rclcpp::init(argc,argv);
	
	std::vector<std::string> camera_name = dv::io::discoverDevices();
	std::vector<rclcpp::Node::SharedPtr> acuro_node;
	
	for(size_t i=0; i<camera_name.size();i++){
		auto node = std::make_shared<Aruco_Detector>(camera_name[i]);
		acuro_node.push_back(node);
	}
	
	std::vector<std::thread> threads;
	
	for(auto &node: acuro_node){
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
