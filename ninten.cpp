#include"ninten.h"
#define LED_PIN 26
#include <ams_as5048b.h>
#include <VL6180X.h>
#include "FaBoMotor_DRV8830.h"
PID PID_a;
PID PID_h;
static AMS_AS5048B as5048(AS5048_ADDRESS+0);
static VL6180X vl6180;
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10
static FaBoMotor hip(0xC0>>1);
static FaBoMotor hand(0xC6>>1);
static FaBoMotor sholder(0xCA>>1);

static float angleMax=240;
static float angle(){
  float r=as5048.angleR(U_DEG, true);
  if(300<r)r-=300;
  return r/angleMax;
}
static float heightMin=18;
static float heightMax=97;
static float height(){
  return (vl6180.readRangeSingleMillimeters()-heightMin)/(heightMax-heightMin);
  
}

void nintenReset(){
  vl6180.init();
  delay(10);
  vl6180.configureDefault();
  delay(10);
  vl6180.setTimeout(500);
  delay(10);
  as5048.begin();
  delay(10);
  
  sholder.begin();
  delay(10);
  hand.begin();
  delay(10);
  hip.begin();
  delay(10);
  sholder.drive(0);
  delay(10);
  hand.drive(0);
  delay(10);
  hip.drive(0);
  delay(10);
}
void nintenSetup(){
  vl6180.init();
  delay(10);
  vl6180.configureDefault();
  delay(10);
  vl6180.setTimeout(1000);
  as5048.begin();

  delay(5);
  
  sholder.begin();
  delay(10);
  hand.begin();
  delay(10);
  hip.begin();
  delay(10);
  pinMode (LED_PIN, OUTPUT);
  digitalWrite(LED_PIN,LOW);  
  sholder.drive(20);
  hip.drive(30);
  delay(2500);
  sholder.drive(0);
  hip.drive(0);
  sholder.drive(0);
  hip.drive(0);
  delay(100);
  as5048.setZeroReg();
  as5048.setZeroReg();
  delay(100);
  while(true){  
    Serial.print("minAngle:");
    Serial.println(as5048.angleR(U_DEG, true), DEC);
    Serial.print("minHeight:");
    Serial.println(heightMin=vl6180.readRangeSingleMillimeters());
    if(fabs(heightMin-18)<10)break;
  }
  sholder.drive(-40);
  hip.drive(-30);
  delay(2500);
  sholder.drive(0);
  hip.drive(0);
  sholder.drive(0);
  hip.drive(0);
  delay(100);

  while(true){
    Serial.print("maxAngle:");
    Serial.println(angleMax=as5048.angleR(U_DEG, true), DEC);
    Serial.print("maxHeight:");
    Serial.println(heightMax=vl6180.readRangeSingleMillimeters());
    if( fabs(angleMax-240)<10 && fabs(heightMax-90)<15)break;
  }
  
  
  digitalWrite(LED_PIN,HIGH);  
  
}

static void clamp(float &v,float minv,float maxv){
  if(v<minv)v=minv;
  if(maxv<v)v=maxv;
}
static float tAngle=0.5;
static float tHeight=0.5;
void nintenLoop(){
  static unsigned long lastMicros;
  unsigned long now=micros();
  int span=now-lastMicros;
  if(span<10*1000)return;
  lastMicros=now;
  float spanf=span*1e-6;
  
  float h=height();
  float a=angle();
  float dA=PID_a.feed(tAngle,a,spanf);
  float dH=PID_h.feed(tHeight,h,spanf);
 /*
  if(count%10==0){
    Serial.print(h);Serial.print("   ");
    Serial.print(a);Serial.print("   ");
    Serial.print(dA);Serial.print("   ");
    Serial.print(dH);Serial.print("   ");
    Serial.println("");
    
  }
  */
  clamp(dA,-56,56);
  clamp(dH,-56,56);
  hip.drive(dA);
  sholder.drive(dH);
}


void setHeight(float in){tHeight=in;}
void setAngle(float in){tAngle=in;}
void setGrip(float in){
  float p=fabs(in-0.5)-0.1;
  if(p<0){
    hand.drive(0);
    return;
  }
  p*=55/0.4f;
  p=round(p);
  if(in<0.5)p*=-1;
  hand.drive(p);
}
