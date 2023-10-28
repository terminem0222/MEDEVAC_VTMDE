//#include <i2c_t3.h>
#include "Arduino.h"
#include "Wire.h"
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "BLE.h"

#define BAUDRATE 115200
#define DT  0.02          // Loop time E.g 0.02 = 20 milliseconds
#define AA  0.97         // complementary filter constant
#define G_GAIN 0.070    // [deg/s/LSB]

byte buff[6];
int accRaw[3];
int magRaw[3];
int gyrRaw[3];
float rate_gyr_y = 0.0;     // [deg/s]
float rate_gyr_x = 0.0;     // [deg/s]
float rate_gyr_z = 0.0;     // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;
float heading = 0.0;
float pkt_num = 0;
float dummy = 0;
/* Gyro calibration */
int magXmax = 1893;
int magYmax = 1023;
int magZmax = 1050;
int magXmin = -1407;
int magYmin = -3101;
int magZmin = -2174;

bool high = true;
struct Packet pkt_payload;

unsigned long startTime;

void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device (initiate again)
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

void setup_berryIMU() 
{
  Wire.begin(); //Init i2c
  Wire.setClock(400000); //Change i2c bus speed to 400k Hz
  Serial.begin(BAUDRATE); //set baudrate to 115200
}

void enable_acclerometer()
{
        //initialise the accelerometer
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10011111);        // ODR 3.33 kHz, +/- 8g , BW = 400hz
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL8_XL,0b11001000);        // Low pass filter enabled, BW9, composite filter
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL3_C,0b01000100);         // Enable Block Data update, increment during multi byte read
}

void enable_magnetometer()
{
  //initialise the magnetometer
  writeTo(LIS3MDL_ADDRESS,LIS3MDL_CTRL_REG1, 0b11011100);      // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
  writeTo(LIS3MDL_ADDRESS,LIS3MDL_CTRL_REG2, 0b00100000);      // +/- 8 gauss
  writeTo(LIS3MDL_ADDRESS,LIS3MDL_CTRL_REG3, 0b00000000);      // Continuous-conversion mode
}

void enable_gyroscope()
{
        //initialise the gyroscope
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL2_G,0b10011100);         // ODR 3.3 kHz, 2000 dps
}

void berryIMU_measure()
{
  startTime = millis();
    //Read the measurements from the sensors, combine and convert to correct values
  //The values are expressed in 2’s complement (MSB for the sign and then 15 bits for the value) 
  //Start at OUT_X_L_A and read 6 bytes.
  readFrom(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_XL, 6, buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
  if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
  if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

  //readMAG(buff);
  readFrom(LIS3MDL_ADDRESS, 0x80 | LIS3MDL_OUT_X_L, 6, buff);
  magRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  magRaw[1] = (int)(buff[2] | (buff[3] << 8));
  magRaw[2] = (int)(buff[4] | (buff[5] << 8));

  /* Added hardiron calib */

  if (magRaw[0] > magXmax)
  {
    magXmax = magRaw[0];
  } 
  if (magRaw[1] > magYmax) 
  {
    magYmax = magRaw[1];
  }
  if (magRaw[2] > magZmax) 
  {
    magZmax = magRaw[2];
  }

  if (magRaw[0] < magXmin) 
  {
    magXmin = magRaw[0];
  }
  if (magRaw[1] < magYmin) 
  {
    magYmin = magRaw[1];
  }
  if (magRaw[2] < magZmin) {
    magZmin = magRaw[2];
  }


  magRaw[0] -= (magXmin + magXmax) /2 ;
  magRaw[1] -= (magYmin + magYmax) /2 ;
  magRaw[2] -= (magZmin + magZmax) /2 ;
  
  readFrom(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_G, 6, buff);
  gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
  gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (gyrRaw[0] >= 32768) gyrRaw[0] = gyrRaw[0]- 65536;
  if (gyrRaw[1] >= 32768) gyrRaw[1] = gyrRaw[1]- 65536;
  if (gyrRaw[2] >= 32768) gyrRaw[2] = gyrRaw[2]- 65536;

  //Convert Gyro raw to degrees per second
  rate_gyr_x = (float) gyrRaw[0] * G_GAIN;
  rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
  rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

  //Calculate the angles from the gyro
  gyroXangle+=rate_gyr_x*DT;
  gyroYangle+=rate_gyr_y*DT;
  gyroZangle+=rate_gyr_z*DT;

    //Convert Accelerometer values to degrees
  AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
  AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;


  //If IMU is up the correct way, use these lines
        AccXangle -= (float)180.0;
  if (AccYangle > 90)
          AccYangle -= (float)270;
  else
    AccYangle += (float)90;

  //Complementary filter used to combine the accelerometer and gyro values.
  CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
  CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;
  dummy = sqrt(pow(CFangleX, 2) + pow(CFangleY, 2));
    //Compute heading  
  heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;
  
  //Convert heading to 0 - 360
  if(heading < 0)
  {
    heading += 360;
  }
            

    //Each loop should be at least DT
  while(millis() - startTime < (DT*1000))
        {
            delay(1);
        }
  //Serial.print( millis()- startTime);

  pkt_payload.CFangleX_data = CFangleX;
  //pkt_payload.CFangleX_data = dummy;
  pkt_payload.gyroXvel_data = rate_gyr_x;

}

float getAccXangle()
{
  return AccXangle;
}

float getAccYangle()
{
  return AccYangle;
}


float getGyroXangle()
{
  return gyroXangle;
}

float getGyroYangle()
{
  return gyroYangle;
}

float getGyroZangle()
{
  return gyroZangle;
}

void dbg_print()
{
//  Serial.print("AccX\t");
//  Serial.print(AccXangle);
//  Serial.print("\t#  AccY\t");
//  Serial.print(AccYangle);
//  Serial.print("\t#  GyrX\t");
//  Serial.print(gyroXangle);
//  Serial.print("\t#  GyrY\t");
//  Serial.print(gyroYangle);
//  Serial.print("\t#  GyrZ\t");
//  Serial.print(gyroZangle);
  //Serial.print("\t# CFangleX\t");
  Serial.println(CFangleX);
//  Serial.print("\t# CFangleY\t");
//  Serial.print(CFangleY);
//  Serial.print("\t# Heading\t ");
//  Serial.println(heading);
}

void setup() {
  // put your setup code here, to run once:
  setup_berryIMU();
  enable_acclerometer();
  enable_magnetometer();
  enable_gyroscope();
  blePayload_setup();

}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("Starting \n");
  berryIMU_measure();
  //delay(3000);
  dbg_print();

  ble_transmit(pkt_payload);
  
  delay(5);

}
