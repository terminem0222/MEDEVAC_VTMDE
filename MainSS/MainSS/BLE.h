#define MAINPROC 1
#define TEENSY 1
#ifdef ARDUINO

  #include "Arduino.h"
  #include <SoftwareSerial.h>

#endif
//https://www.mrswirlyeyes.com/tutorials/bluetooth_hm_10

struct Packet
{
  float CFangleX_data;
  float gyroXvel_data;
  float CFangleZ_data;
  float gyroZvel_data;
  float time_stamp;
};



#ifdef PAYLOAD
void ble_transmit(struct Packet toSend);
#endif

#ifdef MAINPROC
void bleMain_setup();
struct Packet ble_receive();
#endif







