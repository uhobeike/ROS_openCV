//#define BOOST_PYTHON_STATIC_LIB
#include <ros/ros.h>
#include <ros/package.h>
#include "opencv2/opencv.hpp"
#include "iostream"
#include <unistd.h>
#include <fstream>
#include "std_msgs/String.h"

#include <sstream>
//#include <boost/python.hpp> 


using namespace::cv;




cv::Mat gray_img; //グレースケール画像を入れておくためのMat
cv::Mat bin_img; //2値画像を入れておくためのMat
cv::Mat frame; //取得したフレーム
cv::Mat dst;

int px = 0;
int str[469];
int px_cnt;
int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",1000000);
    ros::Rate loop_rate(1000);    
    cv::VideoCapture cap(0);//デバイスのオープン
    //cap.open(0);//こっちでも良い．
    if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
    {
        //読み込みに失敗したときの処理
        return -1;
    }
    
    while(ros::ok())//無限ループ
    {
    	std::stringstream ss;
        cap.read(frame);
	std_msgs::String msg;
        cv::Mat cut_img(frame,cv::Rect(170,0,470,1));               
        cv::cvtColor(cut_img, gray_img, CV_BGR2GRAY); //グレースケールに変換
        cv::threshold(gray_img,bin_img,160,255,THRESH_BINARY); //閾値160で2値画像に変換
        cv::threshold(gray_img, bin_img, 0, 255, THRESH_BINARY | THRESH_OTSU); //閾値を自動で設定
        //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
	//

        //cv::imshow("win", bin_img);//画像を表示．
//	int wh = frame.rows;
//        int ht = frame.cols;

//	std::cout << wh << "," << ht << std::endl;

	//std::cout << "P1" << std::endl;
	//std::cout << bin_img.cols << " " << bin_img.rows << std::endl;

	

    	for(int x = 0; x < bin_img.cols; x++)
    	{
        	px = static_cast<int>(bin_img.at<unsigned char>(0, x));
		if(px == 255){
			px = 1;
    		}
		//std::cout<< px;	
		str[x]=px;
	}
	//std::cout<<std::endl;
	for(int i=198;i<271;i++){
		px_cnt += str[i];
	}
                
	std::cout<<px_cnt<<std::endl;
	ss << px_cnt;
	msg.data = ss.str();
	chatter_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}

