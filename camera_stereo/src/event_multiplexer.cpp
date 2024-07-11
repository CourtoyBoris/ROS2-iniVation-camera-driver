/*
Autor: BLAYES Hugo

This class allows you to recover the packets divided by the talker for each event.
If events arrive too late they are not taken into account.
*/

#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "message/msg/event_batch.hpp"

using namespace std;

using std::placeholders::_1;
using namespace std::chrono_literals;

class EventMultiplexer: public rclcpp::Node{
public:
	EventMultiplexer(string name,string name_cam):Node(name),name_cam(name_cam){
		int NbTopics = 1;
		//this parameter fix the number of event topics
		this->declare_parameter<int>("number_of_topics",1);
		get_parameter("number_of_topics",NbTopics);
		
		//this parameter fix the name of the input event topics 
		declare_parameter<std::string>("event_topic_name","events"); 
  		get_parameter("event_topic_name",topic_name);
		
		//array for the counter to only take the good batch
		for(int i = 0; i<NbTopics; i++){
			nb_topic.push_back(i);
			std::function<void(const message::msg::EventBatch::SharedPtr)> callback = std::bind(&EventMultiplexer::callback,this,_1,i);
			
			rclcpp::Subscription<message::msg::EventBatch>::SharedPtr event = this->create_subscription<message::msg::EventBatch>(topic_name+"_"+name_cam+"_"+std::to_string(i),1000, callback);
			event_sub.push_back(event);
			compteur.push_back(0);
		}
	}




private:
	std::vector<rclcpp::Subscription<message::msg::EventBatch>::SharedPtr> event_sub;
	
	string name_cam;
	string topic_name;
	std::vector<int> compteur;
	std::vector<int> nb_topic;
	
	virtual void topic_callback(const message::msg::EventBatch::SharedPtr msg) = 0;
	
	//callback of the input
	void callback(const message::msg::EventBatch::SharedPtr msg, int topic_number){
		compteur[topic_number]++;
		
		if(compteur[0]!=compteur[topic_number])return;
		
		topic_callback(msg);
	
	}
};
