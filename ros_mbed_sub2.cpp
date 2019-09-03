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

const double enc_per = 0.0005;


//グローバル宣言
int sensval_msg,sensval_msg_LL,sensval_msg_C,enc_flag,enc_cnt,enc_sum,enc_total,trace_flag;
int MotorL=0,MotorR=0;                  //モータPWMデューティ比
int pattern = 0;
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

void messageCb_1(const std_msgs::String& msg_1){
     sensval_msg = atoi(msg_1.data);
    /* else if(sensval_msg <= -80){
         pattern=2;//右レンチェン
     }
     */
}
void messageCb_2(const std_msgs::String& msg_2){
     sensval_msg_LL = atoi(msg_2.data);     
}
void messageCb_3(const std_msgs::String& msg_3){
     sensval_msg_C = atoi(msg_3.data);
}


ros::Subscriber<std_msgs::String> sub_1("chatter_1", &messageCb_1);
ros::Subscriber<std_msgs::String> sub_2("chatter_2", &messageCb_2);
ros::Subscriber<std_msgs::String> sub_3("chatter_3", &messageCb_3);

void LCD_printf(void){
     lcd.setCursor(0,0);
     lcd.printf("%d,%d",sensval_msg,pattern);
     lcd.printf("        ");
     lcd.setCursor(0,1); 
     lcd.printf("%d",sensval_msg_LL);
     lcd.printf("  ");
     lcd.setCursor(2,1); 
     lcd.printf("%d",sensval_msg_C);
     lcd.printf("  ");
     lcd.setCursor(4,1);
     lcd.printf("%d",enc_sum);
     lcd.printf("    ");
     
}

void MotorCtrl(void){
 
                              
    if(MotorL >= 1000) MotorL = 1000;
    if(MotorR >= 1000) MotorR = 1000;
        PWM_A.pulsewidth_us(MotorL);    //左PWM  (0~1000)
        PWM_B.pulsewidth_us(MotorR);    //右PWM  (0~1000)

}
void run_pattern(void){
    
    switch(pattern){
            case 0:
                if(sensval_msg >= 40){
                   pattern=10;//左レンチェン
                   trace_flag=1;//トレース解除
                }
                break;
            case 10://左レンチェン(左白線検知読み飛ばし)
                MotorR = 13;
                MotorL = 13;
                
                if(sensval_msg>=-20 && sensval_msg<=20){//復帰処理
                    trace_flag=0;
                    pattern = 11;
                }
                
                break;
            case 11://左レンチェン(本番)
               
                if(sensval_msg_LL == 0){
                    pattern = 12;
                    trace_flag=1;
                    MotorR = 11;
                    MotorL = 50;
                }
                
                break;
            case 12:
                 MotorR = 11.5;
                 MotorL = 13;
                 if(sensval_msg_LL >= 10&& sensval_msg_C >= 65 && sensval_msg>=-20 && sensval_msg<=20){
                      trace_flag=0;
                      pattern = 13;
                 }
                 break;
             case 13:
                 MotorR = 11;
                 MotorL = 13;
                 /*
                 if(sensval_msg_LL >= 10&& sensval_msg_C >= 65 && sensval_msg>=-20 && sensval_msg<=20){
                      trace_flag=0;
                      pattern = 13;
                 }*/
                 break;
            case 20://右レンチェン
            
                break;
    }


}    
//------------ライントレース--------------------
void LineTrace(void){
    if(trace_flag == 0){
        
        int sensval = 0;
        sensval = sensval_msg;
        
        CommSpeedR = 10;
        CommSpeedL = 13;
        
        
        cp=kp*sensval;
        cd=kd*((before_sensval-sensval)/dt);
        
        cv=cp-cd;
        
        MotorR = int(CommSpeedR - cv);//L
        MotorL = int(CommSpeedL + cv);//R
        
        before_sensval=sensval;
        
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
    run_pattern();
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
    //nh_2.initNode();
    nh.subscribe(sub_1);
    nh.subscribe(sub_2);
    nh.subscribe(sub_3);
    
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
        nh.spinOnce();
        LCD_printf();
        //nh_2.spinOnce();

    }
}
