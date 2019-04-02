// Use No OTA mode for ESP32 
// About 1.5Mbyte memory is used for this sketch 

#include <WiFi.h> 
#include <ArduinoOSC.h>   //https://github.com/hideakitai/ArduinoOSC
#include <ArtnetWifi.h> #// https://github.com/rstephan/ArtnetWifi/
#include "FS.h"
#include "SPIFFS.h"
#include"ninten.h"
#include <ESPmDNS.h>


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#define MIDI_SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define MIDI_CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"
#define DEVIVE_NAME "NintenRobo MIDI"

#include <Wire.h>
#include "esp32_digital_led_lib.h"
#define SDA_PIN 21
#define SCL_PIN 22
#define WS_PIN 33


//Internal DMX data
const uint16_t max_dmx_ch=15;
volatile uint8_t dmx[max_dmx_ch] = {128,128,128,0}; //pan,tilt,grip,RGB*4

/////////////////////////////////////
//// Configuration by SPIFFS
////////////////////////////////////
//configuration on /wifi.txt /pid.txt
char ssid[32];
char wifipw[32];
void readLine(File &fp,char * buf,size_t length,char *name){
  size_t ret=fp.readBytesUntil('\n', buf, length-1);
  buf[ret]=0;
  Serial.print(name);Serial.print(":");Serial.println(buf);
  
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
/////////////////////////////////////
//LED Control
//////////////////////////////////////

strand_t pStrand = {
  .rmtChannel = 0, .gpioNum = WS_PIN , .ledType = LED_WS2812B_V3, 
  .brightLimit = 255, .numPixels = 4,
  .pixels = nullptr, ._stateVars = nullptr};


///////////////////////////////////////
////  WiFi (OSC/Artnet) handler
///////////////////////////////////////

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
//////////////////////////////////////////////
// BLE MIDI handler
/////////////////////////////////////////////

BLEServer *pServer;
BLEAdvertising *pAdvertising;
BLECharacteristic *pCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("BLE MIDI Connected.");
    };
    void onDisconnect(BLEServer* pServer) {
      Serial.println("BLE MIDI Disconnect.");
    }
};
class MyCallbacks: public BLECharacteristicCallbacks {
  void onMidi(byte midi[3]){
    bool silent=false;
    byte ch=midi[1];
    byte v=midi[2];
    if(ch<max_dmx_ch){
      silent=true;
      dmx[ch]=v*2; 
    }else if( 0x2b<=ch &&  ch<=0x48){ //MIDI-keyboard input
      byte mode=ch-0x2b; // 29 mode can seted 
      if(mode<25 && v!=0){
        byte x=mode %5;
        byte y= (mode-x)/5;
        dmx[0]=x*40+27;
        dmx[1]=y*40+27;
        Serial.printf("X:%d Y:%d \n",dmx[0],dmx[1]);
      }
      if(25<=mode && mode <=27){ //RGB Key
        for(int i=mode-25 + 3; i < max_dmx_ch ; i+=3){
          dmx[i]=(v==0)?0:255;
        }
      }
      if(mode==28)for(int i=3;i<max_dmx_ch;i++)dmx[i]=0;
      if(mode==29)for(int i=3;i<max_dmx_ch;i++)dmx[i]=255;
    }
    if(!silent)Serial.printf("MIDI: %02x %02x %02x\n",midi[0],midi[1],midi[2]);    
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    int pos = 0;
    byte midi[3];
    for (int rx = 0; rx < rxValue.length(); rx++)
    {
      byte v=rxValue[rx];
      //Serial.printf("%02x:",v);
      if(v&0x80){
        pos=0;
      }else{
        pos++;
      }
      midi[pos]=v;
      if(pos==2)onMidi(midi);          
    }
  }
};

void midiSetup(){
  BLEDevice::init(DEVIVE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(BLEUUID(MIDI_SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
          BLEUUID(MIDI_CHARACTERISTIC_UUID),
          BLECharacteristic::PROPERTY_READ   |
          BLECharacteristic::PROPERTY_WRITE  |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_WRITE_NR
  );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0x04);
  oAdvertisementData.setCompleteServices(BLEUUID(MIDI_SERVICE_UUID));
  oAdvertisementData.setName(DEVIVE_NAME);
  pAdvertising = pServer->getAdvertising();
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->start();
}



/////////////////////////////
/// Main
////////////////////////////
bool wifiEnabled=true;
void setup() {
  Serial.begin(115200);
  load();
  Wire.begin(SDA_PIN,SCL_PIN);
  nintenReset();
  Wire.begin(SDA_PIN,SCL_PIN);
  nintenReset();
  midiSetup();
  if(ssid[0]==0){
    Serial.println("Wifi disenabled");
    wifiEnabled=false;
  }
  if(wifiEnabled){
    Serial.printf("WiFi start: %s:%s",ssid,wifipw);
    WiFi.begin(ssid,wifipw);
    for(int i=0;i<30;i++) { //wait WiFi connection for 3sec
      if(WiFi.status() != WL_CONNECTED){
        Serial.print("WiFi connected");
        break;
      }
      Serial.print(".");
      delay(100);
    }
  }
  Wire.begin(SDA_PIN,SCL_PIN);
  nintenReset();
  if(wifiEnabled){
    if (!MDNS.begin("ROB")) {
        Serial.println("Error setting up MDNS responder!");
    }
  }

  
  pinMode (WS_PIN, OUTPUT);
  digitalWrite (WS_PIN, LOW);
  if (digitalLeds_initStrands(&pStrand, 1)) {
    Serial.println("Init FAILURE: halting");
    while (true) {};
  }
  digitalLeds_resetPixels(&pStrand);
  nintenSetup();
  if(wifiEnabled){
    MDNS.addService("osc", "udp", 9000);
    oscSetup();
  }
}

void loop() {
  if(wifiEnabled){
    osc.parse();
    artnet.read();
  }
  
  setAngle(dmx[0]/255.0f);
  setHeight(dmx[1]/255.0f);
  setGrip(dmx[2]/255.0f);
  for(int i=0+1;i<4+1;i++){
    pStrand.pixels[i-1]=pixelFromRGB(dmx[i*3],dmx[i*3+1],dmx[i*3+2]);
  }
  digitalLeds_updatePixels(&pStrand);

  
  nintenLoop();
  
}
  
  
