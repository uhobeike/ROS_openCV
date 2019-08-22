#include "mbed.h"
#include <ros.h>
#include <std_msgs/String.h>
#include "AQM0802A.h"
#include <stdlib.h>

ros::NodeHandle nh;
AQM0802A lcd(PB_7, D10);
DigitalOut REFA(D15);
DigitalOut REFB(D14);
DigitalOut ENA(D2);
DigitalOut ENB(D11);
DigitalOut PHA(D3);
DigitalOut PHB(D7);
//DigitalIn sw(PC_13);

PwmOut PWM_A(D5);
PwmOut PWM_B(D4);
//グローバル宣言
int sensval_msg;
int MotorL=0,MotorR=0;                  //モータPWMデューティ比
int CommSpeedR=0,CommSpeedL=0;
int ErrFlg=0;                           //エラー判定フラグ
int before_sensval;
int goal_sens_val = 74;
double cp,cd,cv;
double kp =0.15,kd =0.001,dt=0.001;
//割り込み定義
Ticker flipper;             //汎用タイマー

void messageCb(const std_msgs::String& msg){
     sensval_msg = atoi(msg.data);
     lcd.cls();
     lcd.printf("%d",sensval_msg);
}

ros::Subscriber<std_msgs::String> sub("chatter", &messageCb);
void init(){
    PWM_A.period(0.00005);
    PWM_B.period(0.00005);   
    ////////////前方確認済み値
    REFA = 0;
    ENA = 1;
    PHA = 1;
    REFB = 0;
    ENB = 1;
    PHB = 1;
   
    //sw.mode(PullUp);
 
    nh.initNode();
    nh.subscribe(sub);
}
void MotorCtrl(void){
    
                                  //通常時
        if(MotorL >= 1000) MotorL = 1000;
        if(MotorR >= 1000) MotorR = 1000;
            PWM_A.pulsewidth_us(MotorL);    //左PWM  (0~1000)
            PWM_B.pulsewidth_us(MotorR);    //右PWM  (0~1000)
    
        

}    
//------------ライントレース--------------------
void LineTrace(void){
    int sensval = 0;
    sensval = sensval_msg;
    
    CommSpeedR = 10;
    CommSpeedL = 15;
    
    
    cp=kp*sensval;
    cd=kd*((before_sensval-sensval)/dt);
    
    cv=cp-cd;
    
    MotorR = int(CommSpeedR - cv);//L
    MotorL = int(CommSpeedL + cv);//R
    
    before_sensval=sensval;
    
    //MotorR = 13;
    //MotorL = 13;
}
void flip(){
    //timer++;
    MotorCtrl();
    LineTrace();
}
int main(){
    init();
    //割り込み処理開始  
    flipper.attach(&flip,0.001);              //汎用タイマー割り込み  
    while(1){
        
        nh.spinOnce();
        
    }
}
