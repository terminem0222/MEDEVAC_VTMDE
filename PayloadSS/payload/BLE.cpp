#include "BLE.h"

//http://phillipecantin.blogspot.com/2014/08/hc-05-bluetooth-link-with-zero-code.html

SoftwareSerial mySerial(2, 1); // RX, TX

void sendCommand(const char * command) 
{
  Serial.print("Command send :");
  Serial.println(command);
  mySerial.println(command);
  //wait some time
  delay(100);

  char reply[100];
  int i = 0;
  while (mySerial.available()) {
    reply[i] = mySerial.read();
    i += 1;
  }
  //end the string
  reply[i] = '\0';
  Serial.print(reply);
  Serial.println("Reply end");                 
  delay(50);
}

void writeSerialToBLE(int value) 
{
  mySerial.println(value);
}

void writeToBLE(const char *value) 
{
  Serial.print("Writing :");
  Serial.println(value);
  mySerial.write(value, strlen(value));
}

void readSerial()
{
  char reply[50];
  int i = 0;
  while (mySerial.available()) 
  {
    reply[i] = mySerial.read();
    i += 1;
  }
  //end the string
  reply[i] = '\0';
  if(strlen(reply) > 0)
  {
    Serial.println(reply);
    Serial.println("We have just read some data");
  }
}

void setup_ble_master() 
{
  // put your setup code here, to run once:
  mySerial.begin(9600);
  Serial.begin(9600);

  sendCommand("AT");
  sendCommand("AT+ROLE1");
  sendCommand("AT+UUID0xFFE0");
  sendCommand("AT+CHAR0xFFE1");
  sendCommand("AT+NAMEpayload");
}

void setup_ble_slave() 
{
  // put your setup code here, to run once:
  mySerial.begin(9600);
  Serial.begin(9600);

  sendCommand("AT");
  sendCommand("AT+ROLE0");
  sendCommand("AT+UUID0xFFE0");
  sendCommand("AT+CHAR0xFFE1");
  sendCommand("AT+NAMEreceiver");
}

void sendData(float data)
{
  float temp = data;
  char strFloat[20];
  dtostrf(temp, 2, 2, strFloat);
  writeToBLE(strFloat);
}


