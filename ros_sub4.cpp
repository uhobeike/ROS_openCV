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
InterruptIn enc_rB(D6);

#define speed_curent 0.30573248
double enc_per = 0.0005;

//グローバル宣言
int start_flag;
int end_flag;
int sensval_msg,sensval_msg_LL,sensval_msg_C,sensval_msg_RR,sensval_msg_R,sensval_msg_L,sensval_msg_RLC,enc_flag,trace_flag;
double MotorL=0,MotorR=0;                  //モータPWMデューティ比
int timer_1 = 0,timer_2 = 0;
int sl_cnt = 0,end_cnt = 0;
int pattern = 0;
int CommSpeedR=0,CommSpeedL=0;
int ErrFlg=0;                           //エラー判定フラグ
int before_sensval;
int goal_sens_val = 74;
double enc_cnt,enc_cnt_,enc_sum,enc_total,before_enc,AccelefBefore,target_speed,goal_speed,curent_point,target_point,dev,dif,target_speed_before,Int,iP,iI,iD,enc_before,iRet,enc_jade;
double cp,cd,cv;
double kp =0.10,kd =0.05,dt=0.001;
//割り込み定義
Ticker flipper;             //汎用タイマー
Ticker flipper2;             //汎用タイマー

//Serial pc(USBTX, USBRX);  //これを使っているとrosserial使えないのでコメントアウト

void messageCb_1(const std_msgs::String& msg_1){
     start_flag = 1;
     end_flag = 0;
     end_cnt = 0;
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
void messageCb_4(const std_msgs::String& msg_4){
     sensval_msg_RR = atoi(msg_4.data);     
}
void messageCb_5(const std_msgs::String& msg_5){
     sensval_msg_R = atoi(msg_5.data);
}
void messageCb_6(const std_msgs::String& msg_6){
     sensval_msg_RLC = atoi(msg_6.data);     
}
void messageCb_7(const std_msgs::String& msg_7){
     sensval_msg_L = atoi(msg_7.data);     
}

ros::Subscriber<std_msgs::String> sub_1("chatter_1", &messageCb_1);
ros::Subscriber<std_msgs::String> sub_2("chatter_2", &messageCb_2);
ros::Subscriber<std_msgs::String> sub_3("chatter_3", &messageCb_3);
ros::Subscriber<std_msgs::String> sub_4("chatter_4", &messageCb_4);
ros::Subscriber<std_msgs::String> sub_5("chatter_5", &messageCb_5);
ros::Subscriber<std_msgs::String> sub_6("chatter_6", &messageCb_6);
ros::Subscriber<std_msgs::String> sub_7("chatter_7", &messageCb_7);

void LCD_printf(void){
     lcd.setCursor(0,0);
     lcd.printf("%d,%d",sensval_msg,pattern);
     lcd.printf("        ");
     lcd.setCursor(0,1); 
     lcd.printf("%d",sensval_msg_LL);
     lcd.printf("  ");
     lcd.setCursor(2,1); 
     lcd.printf("%d",sensval_msg_R);
     lcd.printf("    ");
     lcd.setCursor(4,1);
     lcd.printf("%f",enc_total);
     lcd.printf("  ");
     
}

void MotorCtrl(void){
    if(start_flag == 1){
        if(MotorL >= 0)PHA = 1;                         
        
        else if(MotorL < 0){
            PHA = 0;
            MotorL = MotorL * -1; 
        }
        
        if(MotorR >= 0)PHB = 1;
        
        else if(MotorR < 0){
            PHB = 0;
            MotorR = MotorR * -1;
        }                                      
        if(MotorL >= 1000) MotorL = 1000;
        if(MotorR >= 1000) MotorR = 1000;
        PWM_A.pulsewidth_us(MotorL);    //左PWM  (0~1000)
        PWM_B.pulsewidth_us(MotorR);    //右PWM  (0~1000)
        
        end_cnt++;//end_kanri
    }
}
void run_pattern(void){
    
    switch(pattern){
            case 0:
                trace_flag = 0;
                CommSpeedR = 16;
                CommSpeedL = 17;
                if(sensval_msg_L >= 40){
                   enc_total = 0;
                   trace_flag = 1;
                   if(enc_total >=0.01){
                        pattern=90;//左レンチェン
                        break;
                   }
                }
                if(enc_jade == 0&&start_flag==1){
                    sl_cnt++;
                }
                else sl_cnt = 0;
                if(sl_cnt==3000){
                    pattern=30;
                    enc_total = 0;
                }
                if(sensval_msg_RLC >= 550){
                    pattern=40;
                    trace_flag = 1;
                    enc_total = 0;  
                }
                break;
            case 10://左レンチェン(左白線検知読み飛ばし)
                enc_total = 0;
                trace_flag=1;//トレース解除
                MotorR = 16;
                MotorL = 16;
                
                if(enc_total >= 0.06){//復帰処理
                    trace_flag=0;
                    pattern = 11;
                    enc_total = 0;
                }
                break;
            case 11://左レンチェン(本番)
                CommSpeedR = 16;
                CommSpeedL = 17;
                if(enc_jade == 0&&start_flag==1){
                    sl_cnt++;
                }
                else sl_cnt = 0;
                if(sl_cnt==3000){
                    pattern=30;
                    enc_total = 0;
                }
                if(enc_total >= 1.5){
                    pattern = 0;
                }
                if(sensval_msg_C == 0){
                    pattern = 12;
                    trace_flag=1;
                    enc_total = 0;
                    MotorR = 0;
                    MotorL = 0;
                    timer_1 = 0;
                }
                break;
            case 12:
                if(timer_1 >= 1000){
                     MotorR = -19;
                     MotorL = 21;
                     if(enc_total <= -0.045){
                        pattern = 13;
                        enc_sum = 0;
                     }
                }
                 break;
            case 13:
                 MotorR = 15;
                 MotorL = 16;
                 if(enc_total >= 0.3){
                    pattern = 14;
                    trace_flag=0;
                    enc_total = 0;
                 }
                 break;
            case 14:
                CommSpeedR = 16;
                CommSpeedL = 17;
                if(enc_total >= 0.5){
                    pattern = 0;
                }
                break;
                
            case 20://右レンチェン
            
                break;
            case 30://坂処理
                CommSpeedR = 20;
                CommSpeedL = 21;
                trace_flag=0;
                if(enc_total >= 1.550){
                    pattern = 0;
                }
                break;
            case 40://crunk
                MotorR = 14;
                MotorL = 14;
                if(enc_total >= 0.06){
                    pattern = 41;
                    trace_flag = 0;
                    
                }
                break;
            case 41:
                CommSpeedR = 14;
                CommSpeedL = 14;
                if(enc_total >= 0.2 && sensval_msg_R >= 80){
                    pattern = 42;
                    trace_flag = 1;
                    enc_total = 0;
                }
                break;
            case 42:
                MotorR = 14;
                MotorL = 14;
                if(enc_total >= 0.1){
                    pattern = 43;
                    enc_total = 0;
                }
                break;
            case 43:
                MotorR = 24;
                MotorL = 10;
                if(enc_total >= 0.1 && sensval_msg >= -50 && sensval_msg <= 20){
                    pattern = 44;
                    trace_flag = 0;
                    enc_total = 0;
                }
                break;
            case 44:
                CommSpeedR = 14;
                CommSpeedL = 14;
                if(enc_total >= 0.5 && sensval_msg >= -20 && sensval_msg <= 20){
                    pattern = 0;
                }
                break;
            case 90://レンチェン、クランク判断
                if(sensval_msg_RLC >= 550){
                    pattern = 40;
                    enc_total = 0;
                }
                else if(sensval_msg_L >= 40){
                    pattern = 10;
                }
                else if(sensval_msg_R >= 40){
                    pattern = 0;
                }
            case 100://end_stop
                trace_flag = 1;
                MotorR = 0;
                MotorL = 0;
                break;
    }


}    
//------------ライントレース--------------------
void LineTrace(void){
    if(trace_flag == 0){
        
        int sensval = 0;
        sensval = sensval_msg;
        
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
    enc_jade = before_enc - enc_total;
    enc_cnt = 0;
    before_enc = enc_total;
    
}

void timer(){
    timer_1 ++;
    timer_2 ++;
}
void flip(){
    MotorCtrl();
    LineTrace();
    run_pattern();
    enc_process();
    timer();
}
void init(){
    enc_rA.mode(PullUp);
    enc_rB.mode(PullUp);
    PWM_A.period(0.00005);
    PWM_B.period(0.00005);   
    ////////////前方確認済み値
    REFA = 0;
    ENA = 1;
    PHA = 1;//motorL
    REFB = 0;
    ENB = 1;
    PHB = 1;//motorR
   
    //sw.mode(PullUp);
 
    nh.initNode();
    //nh_2.initNode();
    nh.subscribe(sub_1);
    nh.subscribe(sub_2);
    nh.subscribe(sub_3);
    nh.subscribe(sub_4);
    nh.subscribe(sub_5);
    nh.subscribe(sub_6);
    nh.subscribe(sub_7);
    
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
        if(end_cnt >=500){
            pattern = 100;
        }
    }
}
