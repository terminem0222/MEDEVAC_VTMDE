#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_DC 10
#define TFT_CS 9
#define MOSI_PIN 11
#define SCLK_PIN 13

int i = -45;
int Angle = i;
int AngleROC = i;
int CableLength = i;
// String BLEstatus = "CONNECTED";
// String Mode = "BLUETOOTH";
// String stabStatus = "OFF";

bool printMode = 1;
bool printStab = 1;
bool printBLE = 1;


// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void setup() {
  Serial.begin(9600);
  Serial.println("ILI9341 Test!"); 
  SPI.setMOSI(11); //PIN
  SPI.setSCK(13); //PIN
  tft.begin();

  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0,0,320,50, ILI9341_GREENYELLOW);
  tft.fillRect(5,5,310,40, ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);

  tft.setCursor(40,15);
  tft.print("MEDEVAC TEAM");

  tft.setTextSize(2);

  tft.setCursor(5,60);
  tft.print("STABILIZATION STATUS:");

  tft.setCursor(5,90);
  tft.print("SWING ANGLE:");

  tft.setCursor(5,120);
  tft.print("SWING ANGLE ROC:");

  tft.setCursor(5,150);
  tft.print("CABLE LENGTH:");

  tft.setCursor(5,180);
  tft.print("SENSOR MODE:");

  tft.setCursor(5,210);
  tft.print("BLE STATUS:");

  if (printStab == 1){
  tft.fillRect(257,60,36,17, ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(257,60);
  tft.println("ON");
  }
  else{
    tft.fillRect(257,60,36,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(257,60);
    tft.println("OFF");
  }
  //Sensor MODE
  if (printMode == 1){
    tft.fillRect(149,180,110,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(149,180);
    tft.println("BLUETOOTH");
  }
  else{
    tft.fillRect(149,180,110,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(149,180);
    tft.println("OTHER");
  }
  //Bluetooth Status
  if (printBLE == 1){
    tft.fillRect(137,210,144,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(137,210);
    tft.println("CONNECTED");
  }
  else{
    tft.fillRect(137,210,144,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(137,210);
    tft.println("DISCONNECTED");
  }

  tft.setTextColor(ILI9341_RED);
  Serial.begin(9600);
}
void loop(void) {

//Angle
while (i != 45) {
  Angle = i;
  int sizeAngle = String(Angle).length();
  tft.setTextSize(2);
  tft.setCursor(149,90);
  tft.println(Angle);

  tft.setTextSize(1);
  tft.setCursor(149+sizeAngle*12+3,87);
  tft.print("o");

//Angle ROC
  AngleROC = i;
  int sizeAngleROC = String(AngleROC).length();
  tft.setTextSize(2);
  tft.setCursor(197,120);
  tft.println(AngleROC);

  tft.setTextSize(1);
  tft.setCursor(197+sizeAngleROC*12+3,117);
  tft.print("o");
  tft.setTextSize(2);
  tft.setCursor(197+sizeAngleROC*12+8,120);
  tft.print("/sec");

//Cable Length
  CableLength = i;
  int sizeCable = String(CableLength).length();
  tft.setTextSize(2);
  tft.setCursor(161,150);
  tft.println(CableLength);

  tft.setTextSize(2);
  tft.setCursor(161+sizeCable*12+1,150);
  tft.print("ft");

  delay(20);
  tft.fillRect(149,87,100,17, ILI9341_BLACK); //MASK ANGLE
  tft.fillRect(197,117,100,17, ILI9341_BLACK); //MASK ANGLE VEL
  tft.fillRect(161,147,100,17, ILI9341_BLACK);
  i++;
}
i = -45;

  printMode = !printMode;
  printStab = !printStab;
  printBLE = !printBLE;
  //Display Stabilization Status
  if (printStab == 1){
    tft.fillRect(257,60,36,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(257,60);
    tft.println("ON");
  }
  else{
    tft.fillRect(257,60,36,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(257,60);
    tft.println("OFF");
  }
  //Sensor MODE
  if (printMode == 1){
    tft.fillRect(149,180,110,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(149,180);
    tft.println("BLUETOOTH");
  }
  else{
    tft.fillRect(149,180,110,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(149,180);
    tft.println("OTHER");
  }
  //Bluetooth Status
  if (printBLE == 1){
    tft.fillRect(137,210,144,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(137,210);
    tft.println("CONNECTED");
  }
  else{
    tft.fillRect(137,210,144,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(137,210);
    tft.println("DISCONNECTED");
  }

  tft.setTextColor(ILI9341_RED);
}
