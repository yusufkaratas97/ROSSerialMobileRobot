#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "LoRa_E32.h"
#include <SoftwareSerial.h>

ros::NodeHandle  nh;
SoftwareSerial mySerial(10, 11);
LoRa_E32 e32ttl(&mySerial);

float dataXold=0.0;
float dataWold=0.0;

struct Signal {
 float x=0.0;
 float w=0.0;
 //byte countL[4];
 //byte countR[4];
}data;

 

void messageCb( const geometry_msgs::Twist& msg){
  //if(dataXold != data.x || dataWold != data.w)
 // {
 //  data.x = dataXold;
 //  data.w = dataWold;
 // }
 // dataXold = msg.linear.x;
 // dataWold = msg.angular.z;
 data.x = msg.linear.x;
 data.w = msg.angular.z;  
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", messageCb );

void setup()
{
  Serial.begin(9600);
  e32ttl.begin();
  delay(500);
  nh.initNode();
  nh.subscribe(sub);
}
void loop()
{
  nh.spinOnce();
  ResponseStatus rs = e32ttl.sendFixedMessage(0, 44, 23, &data, sizeof(Signal));
  //Serial.println(rs.getResponseDescription());  
  delay(10);

  //while (e32ttl.available()  > 1) {
   // ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
  //  struct Signal data = *(Signal*) rsc.data;
   // Serial.print("left = "); Serial.print(*(int*)data.countL);
   // Serial.print("Right = "); Serial.print(*(int*)data.countR);
  //  rsc.close();
  //}
  }
