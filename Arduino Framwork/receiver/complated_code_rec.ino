#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <Timer.h>

Timer timer;

SoftwareSerial mySerial(50, 51); // Arduino RX <-- e32 TX, Arduino TX --> e32 RX
LoRa_E32 e32ttl(&mySerial);

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

#define LM393_1 30
#define LM393_2 31
#define LM393_3 32
#define LM393_4 33

//int counter[4] = {0,0,0,0};

int counter1 = 0;
int counter2 = 0;
int counter3 = 0;
int counter4 = 0;
int counterR = 0;
int counterL = 0;

float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void diff_driv(float x1, float w1, float *vr1 , float *vl1);
void motor(float xx, float ww);
float DlDrCal(int NumberOfTicks, int Total_Number_of_ticks);

struct Signal {
 float x=0.0;
 float w=0.0;
 //int countL[4];
 //i countR[4];
}data;

const float R = 0.06;
const float l = 0.15;
float vr;
float vl;
//float Dl = 0;
//float Dr = 0;
//float Dc;
//float pi = 3.14;
//float xi = 0.0;
//float yi = 0.0;
//float phii = 0.0; 
//long int LBTick = 0;
//long int LFTick = 0;
//long int RBTick = 0;
//long int RFTick = 0;

//void count1();
//void count2();
//void count3();
//void count4();

void setup() {
  pinMode(LM393_1,INPUT);
  pinMode(LM393_2,INPUT);
  pinMode(LM393_3,INPUT);
  pinMode(LM393_4,INPUT);
  attachInterrupt(digitalPinToInterrupt(LM393_1), count1, RISING);
  //attachInterrupt(digitalPinToInterrupt(LM393_2), count2, RISING);
  //attachInterrupt(digitalPinToInterrupt(LM393_3), count3, RISING);
  //attachInterrupt(digitalPinToInterrupt(LM393_4), count4, RISING);
//   put your setup code here, to run once:
  Serial.begin(9600);
  e32ttl.begin();
  delay(500);
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void loop() {
 // timer.update();
  while (e32ttl.available()  > 1) {
    
  ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    struct Signal data = *(Signal*) rsc.data;
    Serial.println("Gelen Messaj: ");
    Serial.print("the x :  ");
    Serial.println(data.x);
   Serial.print("the w : ");
    Serial.println(data.w);
 aaa
    diff_driv(data.x, data.w, &vr,&vl);
    vr = mapf(abs(vr), 0, 19, 100, 125);
    vl = mapf(abs(vl), 0, 19, 100, 125);
    motor1.setSpeed(vl);
    motor2.setSpeed(vl);
    motor3.setSpeed(vr);
    motor4.setSpeed(vr);
    motor(data.x, data.w);

    counterL = (counter1 + counter2)/2;
    counterR = (counter3 + counter4)/2;
    Serial.print("the counterL :  ");
    Serial.println(counterL);
   Serial.print("the counterR : ");
    Serial.println(counterR);
 
 //   
 //   Dl = DlDrCal(17, 20);
 //   Dr = DlDrCal(17, 20);
 //   Dc = (Dr + Dl)/2;
 //   phii = phii + (Dr - Dl)/l;
 //   xi = xi + Dc * cos(phii);
  //  yi = yi + Dc * sin(phii);

    rsc.close();

//struct Signal {
 //byte x[4];
// byte w[4];
// byte countL[4];
// byte countR[4];
//}data2;
 //  *(int*)data2.countL=counterL;
  // *(int*)data2.countR=counterR;
 
 
  //  ResponseStatus rs = e32ttl.sendFixedMessage(0, 63, 23, &data2, sizeof(Signal));
}
//motor(0.0, 0.0);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
     float result;
     result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
     return result;
}

void diff_driv(float x1, float w1, float *vr1 , float *vl1)
{
  *vr1 = (2*x1 + w1*l)/(2*R);
  *vl1 = (2*x1 - w1*l)/(2*R);

  }

void motor(float xx, float ww)
{

  if(xx > 0 && ww > 0)
  {
    //Serial.println("Forward and left");
    if(xx > ww)
    {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
      counter1++;
      counter2++;
      counter3++;
      counter4++;
      }
    else
    {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
      counter1--;
      counter2--;
      counter3--;
      counter4--;
      }
      
    }
   else if(xx > 0 && ww < 0)
    {
     // Serial.println("forward and right");

    motor1.run(FORWARD);
    motor2.run(FORWARD);

    motor3.run(FORWARD);
    motor4.run(FORWARD);
    counter1++;
      counter2++;
      counter3++;
      counter4++;
    }
    else if(xx < 0 && ww > 0)
    {
     // Serial.println("back and right");
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    counter1--;
      counter2--;
      counter3--;
      counter4--;
    }
   else if(xx < 0 && ww < 0)
   {
   // Serial.println("Backward and left");
    if(xx > ww)
    {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      counter1++;
      counter2++;
      counter3--;
      counter4--;
      }
    else
    {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      counter1--;
      counter2--;
      counter3--;
      counter4--;
      }
    }
    else if(xx > 0 && ww == 0)
    {
     // Serial.println("forward");

    motor1.run(FORWARD);
    motor2.run(FORWARD);

    motor3.run(FORWARD);
    motor4.run(FORWARD);
    counter1++;
      counter2++;
      counter3++;
      counter4++;
    }
    else if(xx < 0 && ww == 0)
    {
    //  Serial.println("back");
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    counter1--;
      counter2--;
      counter3--;
      counter4--;
    }
    else if(xx == 0 && ww > 0)
    {
    // Serial.println("left");
    motor1.setSpeed(vl+10);
    motor2.setSpeed(vl+10);
    motor3.setSpeed(vr+10);
    motor4.setSpeed(vr+10);
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    counter1--;
      counter2--;
      counter3++;
      counter4++;
    }
    else if(xx == 0 && ww < 0)
    {
    //  Serial.println("right");
    motor1.setSpeed(vl+10);
    motor2.setSpeed(vl+10);
    motor3.setSpeed(vr+10);
    motor4.setSpeed(vr+10);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    counter1++;
      counter2++;
      counter3--;
      counter4--;
    }
    else if(xx == 0 && ww == 0)
    {
    //  Serial.println("nothing");
    
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    counter1=0;
      counter2=0;
      counter3=0;
      counter4=0;
    }
    else
    {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    
    motor3.run(RELEASE);
    motor4.run(RELEASE);
    counter1=0;
      counter2=0;
      counter3=0;
      counter4=0;
      }
    
    
    
  }
  
//float DlDrCal(int NumberOfTicks, int Total_Number_of_ticks){
 // float D1;
//  D1 = (2*pi*R * NumberOfTicks) / Total_Number_of_ticks;
 // return D1;

 // }
  void count1() {
  counter1++;
}
  void count2() {
  counter2++;
}
  void count3() {
  counter3++;
}
  void count4() {
  counter4++;
}
