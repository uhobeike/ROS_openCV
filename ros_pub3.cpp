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


cv::Mat gray_img_1; //グレースケール画像を入れておくためのMat
cv::Mat gray_img_2; //グレースケール画像を入れておくためのMat
cv::Mat bin_img_1; //2値画像を入れておくためのMat
cv::Mat bin_img_2; //2値画像を入れておくためのMat
cv::Mat frame; //取得したフレーム
cv::Mat dst;


int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    //ros::NodeHandle n_2;

    ros::Publisher chatter_pub_1 = nh.advertise<std_msgs::String>("chatter_1",1000);
    ros::Publisher chatter_pub_2 = nh.advertise<std_msgs::String>("chatter_2",1000);
    ros::Publisher chatter_pub_3 = nh.advertise<std_msgs::String>("chatter_3",1000);
    ros::Publisher chatter_pub_4 = nh.advertise<std_msgs::String>("chatter_4",1000);
    ros::Publisher chatter_pub_5 = nh.advertise<std_msgs::String>("chatter_5",1000);
    ros::Publisher chatter_pub_6 = nh.advertise<std_msgs::String>("chatter_6",1000);
    ros::Publisher chatter_pub_7 = nh.advertise<std_msgs::String>("chatter_7",1000);

    ros::Rate loop_rate(1000);    
    cv::VideoCapture cap(0);//デバイスのオープン
    //cap.open(0);//こっちでも良い．
    if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
    {
        //読み込みに失敗したときの処理
        return -1;
    }
   
    while(ros::ok()&&cap.read(frame))//無限ループ
    {
       
        int str_1[640] ={};
        int str_2[640] ={};   

        std::stringstream ss_1;
        std::stringstream ss_2;
        std::stringstream ss_3;
        std::stringstream ss_4;
        std::stringstream ss_5;
        std::stringstream ss_6;
        std::stringstream ss_7;
        
        
	    //std_msgs::String msg;
        std_msgs::String msg_1;
        std_msgs::String msg_2;
        std_msgs::String msg_3;
        std_msgs::String msg_4;
        std_msgs::String msg_5;
        std_msgs::String msg_6;
        std_msgs::String msg_7;
        
        cv::Mat cut_img_1(frame,cv::Rect(0,479,640,1));    
        cv::Mat cut_img_2(frame,cv::Rect(0,360,640,1));              
        cv::cvtColor(cut_img_1, gray_img_1, CV_BGR2GRAY); //グレースケールに変換
        cv::threshold(gray_img_1,bin_img_1,170,255,THRESH_BINARY); //閾値160で2値画像に変換
        //cv::threshold(gray_img, bin_img_1, 0, 255, THRESH_BINARY | THRESH_OTSU); //閾値を自動で設定

        cv::cvtColor(cut_img_2, gray_img_2, CV_BGR2GRAY); //グレースケールに変換
        cv::threshold(gray_img_2,bin_img_2,170,255,THRESH_BINARY); //閾値160で2値画像に変換
        //cv::threshold(gray_img, bin_img_2, 0, 255, THRESH_BINARY | THRESH_OTSU); //閾値を自動で設定
	    //cv::imshow("win",bin_img_2);
	    int px_cnt_L = 0;
	    int px_cnt_R = 0;
        int px_cnt_C = 0;
	    int px_cnt_LL = 0;
        int px_cnt_RR = 0;
	    int px_cnt_RLC = 0;
   
        int px_1 = 0;
        int px_2 = 0;

        int sensval_1 = 0;
        int sensval_2 = 0;
        int sensval_3 = 0;
        int sensval_4 = 0;
        int sensval_5 = 0;
        int sensval_6 = 0;
        int sensval_7 = 0;

	    for(int x = 0; x < 640; x++)
	    {
        	px_1 = static_cast<int>(bin_img_1.at<unsigned char>(0, x));
            px_2 = static_cast<int>(bin_img_2.at<unsigned char>(0, x));    	
            if(px_1 == 255){
        		px_1 = 1;
        	}
            if(px_2 == 255){
                px_2 = 1;
            }
	        //std::cout<< px;	
	        str_1[x] = px_1;
            str_2[x] = px_2;
		    /*std::cout << px_2;
            if(x % 40 == 0){
                
                std::cout << std::endl << x << std::endl;
            }
            */
            if(x>=0 && x<640){//crunk
                px_cnt_RLC += str_2[x];
            }            
            if(x>=146 && x<246){//left_change
	            px_cnt_L += str_1[x];
            }
            if(x>=384 && x<484){//right_cahnge
	            px_cnt_R += str_1[x];
            }
            if(x>=246 && x<383){//cennter_judge
                px_cnt_C += str_1[x];
            }
            if(x>=75 && x<125){//most_left_judge
	            px_cnt_LL += str_2[x];
            }
            if(x>=589 && x<640){//most_right_judge
                px_cnt_RR += str_2[x];            
            }
        }
        sensval_1 = px_cnt_L - px_cnt_R;//treace_hensa
	    sensval_2 = px_cnt_LL;	
        sensval_3 = px_cnt_C;
        sensval_4 = px_cnt_RR; 	
        sensval_5 = px_cnt_R;
        sensval_6 = px_cnt_RLC;    
        sensval_7 = px_cnt_L;    
        //std::cout<< sensval_1 << " " << sensval_2 << std::endl;
        ss_1 << sensval_1;
        msg_1.data = ss_1.str();	
	    ss_2 << sensval_2;
	    msg_2.data = ss_2.str();
        ss_3 << sensval_3;
	    msg_3.data = ss_3.str();
        ss_4 << sensval_4;
	    msg_4.data = ss_4.str();
        ss_5 << sensval_5;
	    msg_5.data = ss_5.str();
        ss_6 << sensval_6;
	    msg_6.data = ss_6.str();
        ss_7 << sensval_7;
	    msg_7.data = ss_7.str();

	    chatter_pub_1.publish(msg_1);
        chatter_pub_2.publish(msg_2);
        chatter_pub_3.publish(msg_3);
        chatter_pub_4.publish(msg_4);		
        chatter_pub_5.publish(msg_5);				
        chatter_pub_6.publish(msg_6);
		chatter_pub_7.publish(msg_7);
		        
        ros::spinOnce();
	    loop_rate.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}
