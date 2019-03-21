#include <WiFi.h> 
#include <ArduinoOSC.h>   //https://github.com/hideakitai/ArduinoOSC
#include <ArtnetWifi.h> #// https://github.com/rstephan/ArtnetWifi/
#include "FS.h"
#include "SPIFFS.h"
#include"ninten.h"
#include <ESPmDNS.h>

#include <Wire.h>
#include "esp32_digital_led_lib.h"
#define SDA_PIN 21
#define SCL_PIN 22
#define WS_PIN 33
//configuration on /wifi.txt /pid.txt
char ssid[32];
char wifipw[32];

void readLine(File &fp,char * buf,size_t length,char *name){
  size_t ret=fp.readBytesUntil('\n', buf, length-1);
  buf[ret]=0;
  Serial.print(name);Serial.print(":");Serial.println(name);
  
}
float readLineF(File &fp,char *name){
  char buf[20];
  readLine(fp,buf,sizeof(buf),name);
  if(buf[0]==0)return 0;
  return atof(buf); 
}
int readLineI(File &fp,char *name){
  char buf[20];
  readLine(fp,buf,sizeof(buf),name);
  if(buf[0]==0)return 0;
  return atoi(buf); 
}
void loadWiFi(){ 
  SPIFFS.begin(true);
  File fp = SPIFFS.open("/wifi.txt", FILE_READ);
  readLine(fp,ssid,sizeof(ssid),"ssid");
  readLine(fp,wifipw,sizeof(wifipw),"ssid");
  fp.close();
  SPIFFS.end();  
}
void loadPID(){
  SPIFFS.begin(true);
  File fp = SPIFFS.open("/pid.txt", FILE_READ);
  PID_a.Kp=readLineF(fp,"Ap");
  PID_a.Ki=readLineF(fp,"Ai");
  PID_a.Kd=readLineF(fp,"Ad");
  PID_a.margin=readLineF(fp,"Amargin");
  PID_a.flag=readLineI(fp,"Aflag");

  PID_h.Kp=readLineF(fp,"Hp");
  PID_h.Ki=readLineF(fp,"Hi");
  PID_h.Kd=readLineF(fp,"Hd");
  PID_h.margin=readLineF(fp,"Amargin");
  PID_h.flag=readLineI(fp,"Hflag");
  
  fp.close();
  SPIFFS.end();   
}
void load(){
  loadWiFi();
  loadPID();
}

const uint16_t max_dmx_ch=15;
uint8_t dmx[] = {128,128,128,0}; //pan,tilt,grip,RGB*4

strand_t pStrand = {
  .rmtChannel = 0, .gpioNum = WS_PIN , .ledType = LED_WS2812B_V3, 
  .brightLimit = 255, .numPixels = 4,
  .pixels = nullptr, ._stateVars = nullptr};



ArtnetWifi artnet;
OscWiFi osc;

void onDmx_Artnet(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data){
  if(max_dmx_ch<length)length=max_dmx_ch;
  if(universe==0)for(int i=0;i<length;i++)dmx[i]=data[i];
}
void onDmx_Osc(OscMessage& m)  {
  const char *chStr=m.address().c_str();
  while(*chStr)chStr++;
  while(*chStr!='/')chStr--;
  chStr++;
  unsigned char ch=atoi(chStr);
  float v=m.arg<float>(0);
  if(1.0<v)v=1.0;
  if(v<0)v=0;
  if(ch<max_dmx_ch) dmx[ch]=v*255;
  //mixerIP=m.ip();
  //Serial.print((int)ch); Serial.print(" ");
  //Serial.print(v); Serial.println();
}

void onPID_config(OscMessage& m)  {
  //  /pid/[a,h][p.i,d,f]
  const char *chStr=m.address().c_str();
  PID *p;
  Serial.println(chStr);
  switch(chStr[5]){
    case 'a':
      p=&PID_a;
      break;
    case 'h':
      p=&PID_h;
      break;
    default:
      Serial.println("Invalid channel Ignore");
      return;
  }
  switch(chStr[6]){
    case 'p':
      p->Kp=m.arg<float>(0);
      break;
    case 'i':
      p->Ki=m.arg<float>(0);
      break;
    case 'd':
      p->Kd=m.arg<float>(0);
      break;
    case 'm':
      p->margin=m.arg<float>(0);
      break;
    case 'f':
      p->flag=m.arg<int>(0);
      break;
    default:
      Serial.println("Invalid Channel Ignore");
      return;
  }
  if(chStr[5]=='a' && chStr[6]=='f'){
    Serial.println("PID_a:");
    PID_a.dump();
  }
  if(chStr[5]=='h' && chStr[6]=='f'){
    Serial.println("PID_h:");
    PID_h.dump();
  }
}

void oscSetup(){
  artnet.begin();
  artnet.setArtDmxCallback(onDmx_Artnet);
  osc.begin(9000);
  osc.subscribe("/0/dmx/*", onDmx_Osc);
  osc.subscribe("/pid/*", onPID_config);
}
void setup() {
  Serial.begin(115200);
  load();
  Wire.begin(SDA_PIN,SCL_PIN);
  nintenReset();
  Wire.begin(SDA_PIN,SCL_PIN);
  nintenReset();
  
  WiFi.begin(ssid,wifipw);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
 Wire.begin(SDA_PIN,SCL_PIN);
  nintenReset();
  
  if (!MDNS.begin("ROB")) {
      Serial.println("Error setting up MDNS responder!");
  }
  MDNS.addService("osc", "udp", 9000);

  
  pinMode (WS_PIN, OUTPUT);
  digitalWrite (WS_PIN, LOW);
  if (digitalLeds_initStrands(&pStrand, 1)) {
    Serial.println("Init FAILURE: halting");
    while (true) {};
  }
  digitalLeds_resetPixels(&pStrand);
  oscSetup();
  nintenSetup();
}




void loop() {
  osc.parse();
  artnet.read();
  
  setAngle(dmx[0]/255.0f);
  setHeight(dmx[1]/255.0f);
  setGrip(dmx[2]/255.0f);
  for(int i=0+1;i<4+1;i++){
    pStrand.pixels[i-1]=pixelFromRGB(dmx[i*3],dmx[i*3+1],dmx[i*3+2]);
  }
  digitalLeds_updatePixels(&pStrand);

  
  nintenLoop();
  
}
  
  
