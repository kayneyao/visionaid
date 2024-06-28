#include <Wire.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <TimerOne.h>

int ADXLAddress = 0x53;
int ITGAddress = 0x68;
int HMCAddress = 0x1E;

#define AccX0 0x32
#define AccX1 0x33
#define AccY0 0x34
#define AccY1 0x35
#define AccZ0 0x36
#define AccZ1 0x37
#define AccPWR 0x2D

#define GyroXH 0x1D
#define GyroXL 0x1E
#define GyroYH 0x1F
#define GyroYL 0x20
#define GyroZH 0x21
#define GyroZL 0x
#define GyroPWR 0x3E
#define GyroRNG 0x16

#define MagX0 0x03
#define MagX1 0x04
#define MagY0 0x07
#define MagY1 0x08
#define MagZ0 0x05
#define MagZ1 0x06
#define MagA 0x00
#define MagB 0x01

NMEAGPS  gps;
gps_fix  fix;

long time = 0;
char userInput;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  gpsPort.begin(9600);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(updateTime);

  Wire.beginTransmission(ADXLAddress);
  Wire.write(AccPWR);

  Wire.write(8);
  Wire.endTransmission();

  Wire.beginTransmission(ITGAddress);
  Wire.write(GyroPWR);

  Wire.write(1);
  Wire.endTransmission();

  Wire.beginTransmission(ITGAddress);
  Wire.write(GyroRNG);

  Wire.write(24);
  Wire.endTransmission();

  Wire.beginTransmission(HMCAddress);
  Wire.write(MagA);

  Wire.write(0x18);
  Wire.endTransmission();

  Wire.beginTransmission(HMCAddress);
  Wire.write(MagB);

  Wire.write(0x20);
  Wire.endTransmission();

  delay(300);
}

void loop() {
  // put your main code here, to run repeatedly:
  // if(Serial.available() > 0){

  //   userInput = Serial.read();

  //     if(userInput == 'g'){
        
  //     }
      
  // }
  updateMag();
  updateGyro();
  updateAcc();
  updateGPS();
}

void updateAcc(){
  float X, Y, Z;
  Wire.beginTransmission(ADXLAddress);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXLAddress, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X = ( Wire.read()| Wire.read() << 8); // X-axis value
  X = X/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y = Y/256;
  Z = ( Wire.read()| Wire.read() << 8); // Z-axis value
  Z = Z/256;

  X = X * 9.80665;
  Y = Y * 9.80665;
  Z = Z * 9.80665;

  Serial.print(", ");
  Serial.print(X);
  Serial.print(", ");
  Serial.print(Y);
  Serial.print(", ");
  Serial.print(Z);
}

void updateGyro(){
  float X, Y, Z;
  Wire.beginTransmission(ITGAddress);
  Wire.write(0x1D); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission();

  Wire.requestFrom(ITGAddress, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  X = (Wire.read()*256 + Wire.read()) / 14.375;
  Y = (Wire.read()*256 + Wire.read()) / 14.375;
  Z = (Wire.read()*256 + Wire.read()) / 14.375;

  // Serial.print(", ");
  Serial.print(", ");
  Serial.print(X);
  Serial.print(", ");
  Serial.print(Y);
  Serial.print(", ");
  Serial.print(Z);
}

void updateMag(){
  float X, Y, Z, X1, Y1, Z1;
  Wire.beginTransmission(HMCAddress);
  Wire.write(0x03); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission();

  Wire.requestFrom(HMCAddress, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  
  if(Wire.available()>=6){
    X = (Wire.read()*256 + Wire.read())*0.092;
    Z = (Wire.read()*256 + Wire.read())*0.092;
    Y = (Wire.read()*256 + Wire.read())*0.092;
  }

  // Serial.print(", ");
  Serial.print(X);
  Serial.print(", ");
  Serial.print(Y);
  Serial.print(", ");
  Serial.print(Z);
}

void updateGPS(){
  
  float lat, lng, alt;
  lat = 0.;
  lng = 0.;
  alt = 0.;
  if(gps.available(gpsPort)) {
    lat = fix.latitude();
    lng = fix.longitude();
    alt = fix.altitude();
    time = fix.dateTime_ms();
  }
  Serial.print(", ");
  Serial.print(lat);
  Serial.print(", ");
  Serial.print(lng);
  Serial.print(", ");
  Serial.print(alt);
  Serial.print(", ");
  Serial.println(time);
}

void updateTime(){
  time += 1;
}