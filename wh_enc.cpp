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
PwmOut PWM_A(D5);
PwmOut PWM_B(D4);
InterruptIn enc_rA(D9);
InterruptIn enc_rB(D8);
//InterruptIn enc_lA(D13);
//InterruptIn enc_lB(D12);


//DigitalIn sw(PC_13);

//BusIn in(PA_5, PA_6,PC_13, PC_11);


//QEI qei_left(PA_5, PA_6, NC, 48, QEI::X4_ENCODING);
//QEI qei_right(PC_13, PC_11, NC, 48, QEI::X4_ENCODING);

const double enc_per = 0.0005;


//グローバル宣言
int sensval_msg,enc_flag,enc_cnt,enc_sum,enc_total,pattern,trace_flag ;
int MotorL=0,MotorR=0;                  //モータPWMデューティ比
int CommSpeedR=0,CommSpeedL=0;
int ErrFlg=0;                           //エラー判定フラグ
int before_sensval;
int goal_sens_val = 74;
double cp,cd,cv;
double kp =0.10,kd =0.01,dt=0.001;
//割り込み定義
Ticker flipper;             //汎用タイマー
Ticker flipper2;             //汎用タイマー

//Serial pc(USBTX, USBRX);  //これを使っているとrosserial使えないのでコメントアウト

void messageCb(const std_msgs::String& msg){
     sensval_msg = atoi(msg.data);
     lcd.cls();
     lcd.printf("%d",sensval_msg);
     if(sensval_msg >= 80){
         pattern=1;//左レンチェン
     }
     else if(sensval_msg <= -80){
         pattern=2;//右レンチェン
     }
}

ros::Subscriber<std_msgs::String> sub("chatter", &messageCb);

void MotorCtrl(void){
 
                              //通常時
    if(MotorL >= 1000) MotorL = 1000;
    if(MotorR >= 1000) MotorR = 1000;
        PWM_A.pulsewidth_us(MotorL);    //左PWM  (0~1000)
        PWM_B.pulsewidth_us(MotorR);    //右PWM  (0~1000)

    

}    
//------------ライントレース--------------------
void LineTrace(void){
    if(trace_flag == 0){
        
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
}


void encA_flip_rise(){
    enc_flag |= 0x01;   //0000 0001
    
    if(enc_flag & 0x10) enc_cnt--;  
    else enc_cnt++;
    
} 
void encA_flip_fall(){
    enc_flag &= ~0x01;   //0000 0000
    
    if(enc_flag & 0x10) enc_cnt++;  
    else enc_cnt--;
}
 
void encB_flip_rise(){
    enc_flag |= 0x10;   //0001 0000
    
    if(enc_flag & 0x01) enc_cnt++;  
    else enc_cnt--;
}
void encB_flip_fall(){
    enc_flag &= ~0x10;  //0000 0000
    
    if(enc_flag & 0x01) enc_cnt--;  
    else enc_cnt++;
}
void enc_process(){ //エンコーダーカウント1msごとにリセット
    enc_sum = enc_sum + enc_cnt;
    enc_cnt = enc_cnt * enc_per;
    enc_total = enc_total + enc_cnt;
 
    enc_cnt = 0;
    
}
void flip(){
    
    MotorCtrl();
    LineTrace();
    enc_process();
}
void init(){
    enc_rA.mode(PullUp);
    enc_rB.mode(PullUp);
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
    
    enc_rA.rise(&encA_flip_rise);  //エンコーダーA相立ち上がり割り込み
    enc_rA.fall(&encA_flip_fall);  //エンコーダーA相立ち下り割り込み
    enc_rB.rise(&encB_flip_rise);  //エンコーダーB相立ち上がり割り込み
    enc_rB.fall(&encB_flip_fall);  //エンコーダーB相立ち下り割り込み  
    
    //enc_lA.rise(&encA_flip_rise);  //エンコーダーA相立ち上がり割り込み
    //enc_lA.fall(&encA_flip_fall);  //エンコーダーA相立ち下り割り込み
    //enc_lB.rise(&encB_flip_rise);  //エンコーダーB相立ち上がり割り込み
    //enc_lB.fall(&encB_flip_fall);  //エンコーダーB相立ち下り割り込み  
}
int main(){
    init();
    //割り込み処理開始  
    flipper.attach(&flip,0.001);              //汎用タイマー割り込み    
    while(1){
        
        switch(pattern){
            case 1://左レンチェン
                trace_flag = 1;
                CommSpeedR = 10;
                CommSpeedL = 15;
                if(sensval_msg >=-30 && sensval_msg <=30){
                    trace_flag = 1;
                }
                break;
            case 2://右レンチェン
            
                break;
        }
         
        nh.spinOnce();
    

        
    
        
    }
}
