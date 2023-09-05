//#include <i2c_t3.h>
#include "Wire.h"
#include "LSM9DS0.h"

#define DT  0.02          // Loop time E.g 0.02 = 20 milliseconds
#define AA  0.80         // complementary filter constant
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
  Serial.begin(115200); //set baudrate to 115200
}

void enable_acclerometer()
{
  writeTo(ACC_ADDRESS,CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuos update,  100Hz data rate
  writeTo(ACC_ADDRESS,CTRL_REG2_XM, 0b00100000); // +/- 16G full scale
}

void enable_gyroscope()
{
  // Enable Gyro
  writeTo(GYR_ADDRESS, CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
  writeTo(GYR_ADDRESS, CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale
}

void berryIMU_measure()
{
    //Read the measurements from the sensors, combine and convert to correct values
  //The values are expressed in 2’s complement (MSB for the sign and then 15 bits for the value) 
  //Start at OUT_X_L_A and read 6 bytes.
  readFrom(ACC_ADDRESS, 0x80 | OUT_X_L_A, 6, buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
  if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
  if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

  readFrom(GYR_ADDRESS, 0x80 | OUT_X_L_G, 6, buff);
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

void setup() {
  // put your setup code here, to run once:
  setup_berryIMU();
  enable_acclerometer();
  enable_gyroscope();

}

void loop() {
  // put your main code here, to run repeatedly:
  berryIMU_measure();

}
