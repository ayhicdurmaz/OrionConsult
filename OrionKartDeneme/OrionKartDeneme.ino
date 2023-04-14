#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BME280.h>
#include <TinyGPSPlus.h>
#include "MovingAverageFloat.h"

#define LORA  
#define BME_ADRESS_ALTERNATE (0X76)

Adafruit_BME280 bme;
float base_pressure;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

TinyGPSPlus gps;
byte satVal;
float latitude;
float longtitude;
String strLatitude = "0.0000000";
String strLongtitude = "0.0000000";

String data = "";
MovingAverageFloat <10> filter;
float pressure, altitude, curAlt,angle_x, angle_y, offsetX, offsetY, g;

float loraTimer = 0;

int fallCounter = 0;

bool isApogee = false;

void setup(){
  Serial.begin(115200);
  pinMode(40,OUTPUT);
  //GPS
  Serial.println("GPS ve Lora");
  Serial3.begin(9600); // Bu GPS
  Serial1.begin(115200); // Bu lora

  //BME280
  Serial.println("BME");
  Wire.begin();
  bme.begin(BME_ADRESS_ALTERNATE); 
  delay(1000); 
  base_pressure = bme.readPressure() / 100.0F;

  //BNO055
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  calcOffset();

}

void loop(){
  readGPS();

  altitude = filter.add((float) ((int)(bme.readAltitude(base_pressure) * 10)) / 10.0 );
  
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  g = sqrt( pow(gravity.y(),2) + pow(gravity.z(),2) );
  float x = abs(euler.x());
  angle_x = x > 180 ? 360 - x - offsetX : x - offsetX;  
  angle_y = euler.y() - offsetY;

  apogeeDetector();

  if(millis() - loraTimer > 500){
    altitude = filter.add((float) ((int)(bme.readAltitude(base_pressure) * 10)) / 10.0 );
    curAlt = altitude;
    sendData();  
    loraTimer = millis();
  }
  
  curAlt = altitude;
}

void apogeeDetector(){
  if(g > 9.0){
    isApogee = true;
    analogWrite(40,150);
  }
  // if(abs(angle_x) > 65 || abs(angle_y) > 65){
  //   isApogee = true;
  //   analogWrite(40,150);
  // }

  if(altitude > 2){
    if(curAlt > altitude){
      fallCounter++;        
    }
    else if(curAlt == altitude){

    }
    else{
      fallCounter = 0;
    }

    if(fallCounter >= 10){
      isApogee = true;
      analogWrite(40,150);
    }
  }
}

void calcOffset(){
  for(int i = 0; i < 1000; i++){
    imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    int x = abs(euler.x());
    offsetX += x > 180 ? 360 - x : x;
    offsetY += euler.y();      
  }  
  offsetX /= 1000;
  offsetY /= 1000;
}

void setData(){
  data = "*,";
  data += altitude; data += ",";
  data += angle_x; data += ",";
  data += angle_y; data += ",";
  data += g; data += ",";
  data += satVal; data += ",";
  data += strLatitude; data += ",";
  data += strLongtitude; data += ",";
  data += isApogee;
}

void sendData(){
  setData();
#ifdef LORA
  Serial1.println(data);
  Serial1.flush();
#else
  Serial.println(data);
  Serial.flush();
#endif
} 

void readGPS() {
  while (Serial3.available() > 1)
  {
    gps.encode(Serial3.read());
    satVal = gps.satellites.value();
    if (gps.location.isUpdated())
    {
      latitude = gps.location.lat();
      longtitude = gps.location.lng();
      strLatitude = String(latitude, 6);
      strLongtitude = String(longtitude, 6);
    }
  }
}