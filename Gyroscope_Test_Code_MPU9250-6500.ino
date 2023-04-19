#include <Wire.h>
#include <MPU6050_light.h>


MPU6050 mpu(Wire);
void setup() {
  Serial.begin(9600);
  Wire.begin();

  //initializes gyroscope
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  
}

void loop() {
  mpu.update();

  Serial.print("AngleZ: ");
  Serial.print(mpu.getAngleZ());
  Serial.print("    AngleZ Acc: ");
  Serial.println(mpu.getGyroZ());

}
