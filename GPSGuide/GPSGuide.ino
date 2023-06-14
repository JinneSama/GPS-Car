#include <Wire.h>
#include<TinyGPS++.h>
#include<SoftwareSerial.h>
#include"ServoTimer2.h"
#include <LiquidCrystal_I2C.h>
#include <HMC5883L.h>

HMC5883L compass;

float latc,logc;
float latd=16.320794;
float logd=121.126726;
float bearing;
float heading;

const int rp = 4;
const int rn = 5;
const int speedcn = 9;
const int servoPin = 10;

int dir = 3;
bool moveForward = false;


String msg;
int index;
String longmsg;
String latmsg;

SoftwareSerial GPS_SoftSerial(2,3);
TinyGPSPlus gps;
ServoTimer2 mservo;
//LiquidCrystal_I2C lcd(0x27,20,4);

void setup() 
{
  Serial.begin(115200);
  pinMode(rp,OUTPUT);
  pinMode(rn,OUTPUT);
  pinMode(speedcn,OUTPUT);
  mservo.attach(servoPin);
  
  mservo.write(105);
  GPS_SoftSerial.begin(9600); 
  setupCompass();

  //lcd.init();
  //lcd.init();
  //lcd.backlight();
  //lcd.clear();
  //lcd.setCursor (0,0);
  //lcd.print("System Starting....");
  
  while (!Serial) {
    ;
  }
  delay(2000);
  //lcd.clear();
}

void loop() 
{
  readSerial();
  headingcal();
  gpsdata();
  gpsheading();
  steering();
}

void readSerial(){
  if (Serial.available()) {
    msg = Serial.readStringUntil('\n');
    index = msg.indexOf('-');
    longmsg = msg.substring(0 , index);
    latmsg = msg.substring(index + 1 , msg.length());
    latd = latmsg.toFloat();
    logd = longmsg.toFloat();
  }
  
  //lcd.setCursor(0,2);
  //lcd.print(latd , 5);
  //lcd.setCursor(0,3);
  //lcd.print(logd ,5);
}

void setupCompass(){
  while (!compass.begin())
  {
    delay(500);
  }
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(212 , -440);
}

void gpsdata()
{
  smartDelay(100);
  unsigned long start;
  double lat_val, lng_val, alt_m_val;
  bool loc_valid, alt_valid;
  lat_val = gps.location.lat(); 
  loc_valid = gps.location.isValid(); 
  lng_val = gps.location.lng();
  alt_m_val = gps.altitude.meters(); 
  alt_valid = gps.altitude.isValid(); 
  
  if (!loc_valid)
  {
    moveForward = false;
  }else{
    moveForward = true;
  }
  
  latc=lat_val;
  logc=lng_val;  
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS_SoftSerial.available()) 
    gps.encode(GPS_SoftSerial.read());
  } while (millis() - start < ms);
}

void gpsheading()
{
  float x,y,deltalog,deltalat;
  deltalog= logd-logc;
  deltalat=latd-latc;

  //calculation for bearing
  x=cos(latd)*sin(deltalog);
  y=(cos(latc)*sin(latd))-(sin(latc)*cos(latd)*cos(deltalog));
  bearing=(atan2(x , y)) * (180/3.14);
  bearing = fmodf((360 - bearing),360);
  //lcd.setCursor(0,0);
  //lcd.print("B: ");
  //lcd.setCursor(3,0);
  //lcd.print(bearing , 1);

  //calculation for distance between the 2 points
  float a,d,c;
  a=(((sin(deltalat/2)))*(sin(deltalat/2))) + ((cos(latc))*(cos(latd))* (((sin(deltalog/2)))*(sin(deltalog/2)))  );
  c=2*(atan2(sqrt(a),sqrt(1-a)));
  d=6371*c; 

  //lcd.setCursor(8,1);
  //lcd.print("D: ");
  //lcd.setCursor(11,1);
  //lcd.print(d , 4);
}

void headingcal()
{
  Vector norm = compass.readNormalize();

  heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }
   
  heading = heading * 180/M_PI; 
  //lcd.setCursor(12,0);
  //lcd.print("H: ");
  //lcd.setCursor(15,0);
  //lcd.print(heading , 1);
}


void steering()
{
  float finalv;
  finalv = heading/bearing;
  
  //lcd.setCursor(0,1);
  //lcd.print("F: ");
  //lcd.setCursor(3,1);
  //lcd.print(finalv , 2);
    
  if(finalv>=0.88&&finalv<=1.12)
  {
    if(dir != 0){
      mservo.write(1800);
      forward();
    }
    dir = 0;
  }else if(finalv >1.12)
  {
    if(dir != 1){
      mservo.write(2250);
      forward();
    }
    dir = 1;
  }
  else if(finalv < 0.88)
  {
    if(dir != 2){
      mservo.write(750);
      forward();
    }
    dir = 2;
  }
  else if((logd==logc) && (latc==latd))
  {
    if(dir != 0){
      mservo.write(105);
      stop1();
    }
    dir = 0;
  }
}

void forward()
{
  if(moveForward){    
    analogWrite(speedcn , 20);
    digitalWrite(rp,HIGH);
    digitalWrite(rn,HIGH);
  }
}

void stop1()
{
  analogWrite(speedcn , 0);
  digitalWrite(rp,LOW);
  digitalWrite(rn,LOW);
}
