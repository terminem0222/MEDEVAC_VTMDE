#include "Arduino.h"
#include <SoftwareSerial.h>

//http://phillipecantin.blogspot.com/2014/08/hc-05-bluetooth-link-with-zero-code.html



void sendCommand(const char * command);

void writeSerialToBLE(int value);

void writeToBLE(const char *value);

void readSerial();

void setup_ble_master();

void setup_ble_slave();

void sendData(float data);

