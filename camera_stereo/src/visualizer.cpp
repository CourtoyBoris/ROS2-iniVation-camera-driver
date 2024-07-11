/*
Autor: BLAYES Hugo

Visualizer to see events and image wihout timestamp contraint
*/

#include  <opencv2/opencv.hpp>
#include <string>
#include <iostream>

using namespace std;

class Visualizer{
public:


	Visualizer(){
		resolution = {0,0};
		image = cv::Mat::zeros(0,0,CV_8U);
	}
	
	void changeResolution(int x, int y){
		resolution = {x,y};
		image = cv::Mat::zeros(x,y,CV_8U);		
	}
	
	void changeName(string w){
		name = w;
		c = name.c_str();
	}
	
	void afficher(){
		cv::imshow(c,image);
		cv::waitKey(1);
		reset();
	}
	
	void reset(){
		image = cv::Mat::zeros(resolution.height, resolution.width,CV_8U);
		affichage_compteur = 0;
		affichage_compteur_solo = 0;
	}
	
	void addMatrice(const cv::Mat mat){
		image = mat;
	
		affichage_compteur++;
		if(affichage_compteur > affichage_max){
			afficher();
		}
	}
	
	void addEvents(const std::vector<short int> x, const std::vector<short int> y, const std::vector<short int> polarity){
		for(size_t i = 0; i < x.size(); i++){
			if(polarity[i] == 1){
				image.at<unsigned char>(x[i],y[i]) = 128; 
			}else if(polarity[i] == 0){
				image.at<unsigned char>(x[i],y[i]) = 255;
			}
		}
		
		affichage_compteur++;
		if(affichage_compteur > affichage_max){
			afficher();
		}
	}
	
	void addEvent(const int x,const int y,const int polarity){
		if(polarity == 1){
			image.at<unsigned char>(x,y) = 128;
		}else if(polarity == 0){
			image.at<unsigned char>(x,y) = 255;
		}
		
		affichage_compteur_solo++;
		if(affichage_compteur_solo > affichage_max_solo){
			afficher();
		}
	}
	
	
	
private:
	cv::Mat image;
	cv::Size resolution;
	
	int affichage_compteur = 0;
	int affichage_max = 10;
	int affichage_compteur_solo = 0;
	int affichage_max_solo = 33000;
	
	string name;
	const char* c;
};
