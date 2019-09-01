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
int sensval_1;
int sensval_2;
int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub_1 = n.advertise<std_msgs::String>("chatter_1",1000);
    ros::Publisher chatter_pub_2 = n.advertise<std_msgs::String>("chatter_2",1000);

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
        //std::stringstream ss_2;
        
        cap.read(frame);
	    //std_msgs::String msg;
        std_msgs::String msg_1;
        std_msgs::String msg_2;
        cv::Mat cut_img(frame,cv::Rect(170,0,470,1));               
        cv::cvtColor(cut_img, gray_img, CV_BGR2GRAY); //グレースケールに変換
        cv::threshold(gray_img,bin_img,160,255,THRESH_BINARY); //閾値160で2値画像に変換
        cv::threshold(gray_img, bin_img, 0, 255, THRESH_BINARY | THRESH_OTSU); //閾値を自動で設定
        //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
	//

        cv::imshow("win", bin_img);//画像を表示．
//	int wh = frame.rows;
//        int ht = frame.cols;

//	std::cout << wh << "," << ht << std::endl;

	//std::cout << "P1" << std::endl;
	//std::cout << bin_img.cols << " " << bin_img.rows << std::endl;

	
	int px_cnt_L = 0;
	int px_cnt_R = 0;
	int px_cnt_LL = 0;
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
	for(int i=0;i<20;i++){
		px_cnt_LL += str[i];
	}
	for(int i=107;i<198;i++){
		px_cnt_L += str[i];
	}
	for(int i=271;i<361;i++){
		px_cnt_R += str[i];
	}
    sensval_1 = px_cnt_L - px_cnt_R;
	sensval_2 = px_cnt_LL;	
	std::cout<< sensval_1 <<std::endl;
    ss << sensval_1;
    msg_1.data = ss.str();	
	ss << sensval_2;
	msg_2.data = ss.str();
	chatter_pub_1.publish(msg_1);
    chatter_pub_2.publish(msg_2);
	ros::spinOnce();
	loop_rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}
