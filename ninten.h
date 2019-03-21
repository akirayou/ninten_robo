#ifndef _NINTEN_H_
#define _NINTEN_H_
#define LED_PIN 26
#define  PID_FLAG_CLEAR_I 1
#include <Arduino.h>
void nintenSetup();
void nintenLoop();
void setHeight(float in);
void setAngle(float in);
void setGrip(float in);
void nintenReset();
class PID{
  public:
  float Kp;
  float Ki;
  float Kd;
  float margin;
  int flag;

  //flowing is for internal use but public for debugging
  float p;
  float i;
  float d;
  float oldP;

  public:
  void init(){
    p=0;
    i=0;
    d=0;
    oldP=0;
  }
  PID(){
    init();  
  }
  float feed(float target,float now,float span){
    p=target-now;
    if(margin>fabs(p)){
      p=0;
    }
    i+=p*span;
    d=(p-oldP)/span;
    if(PID_FLAG_CLEAR_I & flag && i*p<0)i=0;
    oldP=p;
    return Kp*p+Ki*i+Kd*d;
  }
  void dump(){
    Serial.print("Kp:");Serial.println(Kp);
    Serial.print("Ki:");Serial.println(Ki);
    Serial.print("Kd:");Serial.println(Kd);
    Serial.print("margin:");Serial.println(margin);
    Serial.print("flag:");Serial.println(flag);
  }

  
};
extern PID PID_a;
extern PID PID_h;

#endif
