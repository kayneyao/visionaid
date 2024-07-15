#include <NMEAGPS.h>
#include <GPSport.h>
#include <TimerOne.h>

#include "MPU9250.h"

#define I2Cclock 400000
#define I2Cport Wire

#define AHRS false
#define SerDebug false

#define MPU9250_ADDRESS 0x68

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

NMEAGPS  gps;
gps_fix  fix;

long time = 0;

// float magBias[3] = {-28.30, 353.16, -18.88};
// float magScale[3] = {1.14, 0.90, 0.99};

char userInput;

void initIMU() {
  // Read the WHO_AM_I register, this is a good test of communication
  // myIMU.magBias[0] = -28.30;
  // myIMU.magBias[1] = 353.16;
  // myIMU.magBias[2] = -18.88;
  // myIMU.magScale[0] = 1.14;
  // myIMU.magScale[1] = 0.90;
  // myIMU.magScale[2] = 0.99;

  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if(SerDebug){
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);
  }

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    // Start by performing self test and reporting values
    if(SerDebug){
      Serial.println(F("MPU9250 is online..."));
      Serial.print(F("x-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: acceleration trim within : "));
      Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
      Serial.print(F("x-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
      Serial.print(F("y-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
      Serial.print(F("z-axis self test: gyration trim within : "));
      Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    // myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
      

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if(SerDebug){
      Serial.println("MPU9250 initialized for active data mode....");
      Serial.print("AK8963 ");
      Serial.print("I AM 0x");
      Serial.print(d, HEX);
      Serial.print(" I should be 0x");
      Serial.println(0x48, HEX);
    }

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    

    if (SerDebug)
    {
      Serial.println("AK8963 initialized for active data mode....");
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    // myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    // Serial.println("AK8963 mag biases (mG)");
    // Serial.println(myIMU.magBias[0]);
    // Serial.println(myIMU.magBias[1]);
    // Serial.println(myIMU.magBias[2]);

    // Serial.println("AK8963 mag scale (mG)");
    // Serial.println(myIMU.magScale[0]);
    // Serial.println(myIMU.magScale[1]);
    // Serial.println(myIMU.magScale[2]);
    delay(2000); // Add delay to see results before serial spew of data

    if(SerDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    byte mag = myIMU.readByte(AK8963_ADDRESS, GYRO_CONFIG);
    byte gyro = myIMU.readByte(MPU9250_ADDRESS, GYRO_CONFIG);
    byte acc = myIMU.readByte(MPU9250_ADDRESS, GYRO_CONFIG);
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  gpsPort.begin(9600);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(updateTime);

  while(!Serial){}

  initIMU();

  delay(1000);
}

void readData(){
//Check interupt pin for update
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read Accel

//Raw Data -> Real World Units
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;

    myIMU.readGyroData(myIMU.gyroCount);  // Read Gyro

    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read Magnet

    myIMU.mx = ((float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0]) * 0.092;
    myIMU.my = ((float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1]) * 0.092;
    myIMU.mz = ((float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2]) * 0.092;
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  while(gps.available(gpsPort)){
    fix = gps.read();
  }
}

void printRaw(){
  Serial.print((float)myIMU.mx); Serial.print(", ");
  Serial.print((float)myIMU.my); Serial.print(", ");
  Serial.print((float)myIMU.mz); Serial.print(", ");

  Serial.print((float)myIMU.gy); Serial.print(", ");
  Serial.print((float)myIMU.gx); Serial.print(", ");
  Serial.print((float)myIMU.gz); Serial.print(", ");

  Serial.print((float)myIMU.ay); Serial.print(", ");
  Serial.print((float)myIMU.ax); Serial.print(", ");
  Serial.print((float)myIMU.az); Serial.print(", ");
}

void printMag(){
  Serial.print((float)myIMU.mx); Serial.print(",");
  Serial.print((float)myIMU.my); Serial.print(",");
  Serial.println((float)myIMU.mz);
}

void printAcc(){
  Serial.print((float)myIMU.ay); Serial.print(",");
  Serial.print((float)myIMU.ax); Serial.print(",");
  Serial.println((float)myIMU.az);
}

void updateTime(){
  time += 1;
}

void updateGPS(){
  

  Serial.print(fix.location.lat());
  Serial.print(", ");
  Serial.print(fix.location.lon());
  Serial.print(", ");
  Serial.print(fix.alt.int32_000());
  Serial.print(", ");
  Serial.println(time);
}

void printYPR(){
//   myIMU.delt_t = millis() - myIMU.count;

// //Quaternion to Euler Angles
//   myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
//                 * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
//                 * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
//                 * *(getQ()+3));
//   myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
//                 * *(getQ()+2)));
//   myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
//                 * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
//                 * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
//                 * *(getQ()+3));
// //Radian to Degree
//   myIMU.pitch *= RAD_TO_DEG;
//   myIMU.yaw   *= RAD_TO_DEG;
//   myIMU.yaw   += 0.085768519; //Magnetic Declination in Radians
//   myIMU.roll  *= RAD_TO_DEG;
// //YPR output
//   Serial.print(myIMU.yaw);
//   Serial.print(", ");
//   Serial.print(myIMU.pitch);
//   Serial.print(", ");
//   Serial.println(myIMU.roll);

//   myIMU.count = millis();
//   myIMU.sumCount = 0;
//   myIMU.sum = 0; // if (myIMU.delt_t > 500)
}

void printQuat(){

}


void loop() {
  // put your main code here, to run repeatedly:
  // Wire.requestFrom(0x69, 1);
  
  // byte b = Wire.read();

  readData();
  

  // myIMU.updateTime();

  // MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
  //                        myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
  //                        myIMU.mx, myIMU.mz, myIMU.deltat);
  // printMag();
  // delay(100);
  if(Serial.available()){

    userInput = Serial.read();

    if(userInput == 'g'){
      printRaw();
      updateGPS();
    }
      
  }
  // while(gpsPort.available() > 0){
  //   byte read = gpsPort.read();
  //   Serial.write(read);
  // }
  
  
  // Serial.print(a); Serial.print(", ");
  // Serial.println(b);
}
