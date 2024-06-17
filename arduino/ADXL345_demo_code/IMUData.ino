#include <Wire.h>
#include <ADXL345.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "ITG3200.h"

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
ITG3200 gyro;

int error = 0;

void setup(){
  Serial.begin(9600);

  gyro.init();
  gyro.zeroCalibrate(200,10);//sample 200 times to calibrate and it will take 200*10ms
  
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
}

void loop(){
  sensors_event_t event; 
  mag.getEvent(&event);

  float gx,gy,gz;
  gyro.getAngularVelocity(&gx,&gy,&gz);

	double xyz[3];
	double ax,ay,az;
	adxl.getAcceleration(xyz);
	ax = xyz[0];
	ay = xyz[1];
	az = xyz[2];
	Serial.print(event.magnetic.x);
  Serial.print(", ");
  Serial.print(event.magnetic.y);
  Serial.print(", ");
  Serial.print(event.magnetic.z);
  Serial.print(", ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gz);
  Serial.print(", ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);
	delay(100);
 
}
