#include "SD.h"
#include <SPI.h>
#include "BLE.h"

void setup_SD();
void writeToSD(struct Packet pkt_mainrx, uint8_t bootmode, long long sample);
