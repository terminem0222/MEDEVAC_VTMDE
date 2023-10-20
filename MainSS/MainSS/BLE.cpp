#include "BLE.h"

//https://www.mrswirlyeyes.com/tutorials/bluetooth_hm_10
#define BLE_BAUDRATE  115200
#ifdef ARDUINO
SoftwareSerial payloadBTSerial(2, 3); // RX, TX
#endif

struct Packet pkt_tx;
struct Packet pkt_rx;

#ifdef PAYLOAD
void blePayload_setup()
{
  payloadBTSerial.begin(BLE_BAUDRATE);
}
void ble_transmit(struct Packet toSend)
{
  pkt_tx.CFangleX_data = toSend.CFangleX_data;
  pkt_tx.gyroXvel_data = toSend.gyroXvel_data;
  pkt_tx.CFangleX_data = toSend.CFangleY_data;
  pkt_tx.gyroXvel_data = toSend.gyroYvel_data;
  pkt_tx.CFangle_data = toSend.CFangle_data;
  pkt_tx.gyrovel_data = toSend.gyrovel_data;

  payloadBTSerial.write((byte *) & pkt_tx,sizeof(Packet));
  /*
  Serial.print("TX: ");
  //Serial.print(" ");
  //Serial.print(pkt_tx.gyroXvel_data);
  Serial.print(" ");
  Serial.print(pkt_tx.CFangle_data);
  Serial.print(" ");
  Serial.println(pkt_tx.gyrovel_data);
  */
}
#endif

#ifdef MAINPROC
void bleMain_setup()
{
  Serial6.begin(BLE_BAUDRATE);
}

struct Packet ble_receive()
{
// Counting variable to fix a lost connection
  static byte count = 10;
  
  // Check the software serial buffer for data to read
  if((uint) Serial6.available() >= sizeof(Packet)) 
  {
    // Read in the appropriate number of bytes to fit our Packet
    Serial6.readBytes((byte *) & pkt_rx,sizeof(Packet));

    /*
    // Print the Packet contents
    Serial.print("RX: ");
    Serial.print(pkt_rx.CFangleX_data);
    Serial.print(" ");
    Serial.print(pkt_rx.gyroXvel_data);
    Serial.print(" ");
    Serial.print(pkt_rx.CFangle_data);
    Serial.print(" ");
    Serial.println(pkt_rx.gyrovel_data);    
    */

    //pkt_mainrx.CFangleX_data = pkt_rx.CFangleX_data;
    //pkt_mainrx.gyroXvel_data = pkt_rx.gyroXvel_data;
    //Serial.println(pkt_mainrx.CFangleX_data);
    // Flush the serial buffer
    while(Serial6.available() > 0)
      Serial6.read();   
    
    // Transmit data via bluetooth
    //ble_transmit();
  } 
  else 
  {
    // If a disconnect happens, start transmitting
    if(count >= 10) 
    {
      count = 0;  // Reset counter
      // Transmit to revive process
      //ble_transmit();
    }
    count++;
  }

  //pkt_rx.CFangle_data = sqrt(pow(pkt_rx.CFangleX_data, 2) + pow(pkt_rx.CFangleY_data, 2));
  //pkt_rx.gyrovel_data = sqrt(pow(pkt_rx.gyroXvel_data, 2) + pow(pkt_rx.gyroYvel_data, 2));
  return pkt_rx;
}
#endif