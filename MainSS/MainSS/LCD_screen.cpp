#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_DC 10
#define TFT_CS 9
#define MOSI_PIN 11
#define SCLK_PIN 13

#define SENSOR_MODE 1
#define BLE_CON     1

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void setup_LCD()
{
  Serial.println("ILI9341 Init");
  SPI.setMOSI(MOSI_PIN);
  SPI.setSCK(SCLK_PIN);
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

}

void LCD_printStabOn()
{
  tft.fillRect(257,60,36,17, ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(257,60);
  tft.println("ON");
}

void LCD_printStabOff()
{
    tft.fillRect(257,60,36,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(257,60);
    tft.println("OFF");
}

void LCD_printSensorMode()
{
  if(SENSOR_MODE)
  {
    tft.fillRect(149,180,110,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(149,180);
    tft.println("BLUETOOTH");
  }
  else if (!SENSOR_MODE)
  {
    tft.fillRect(149,180,110,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(149,180);
    tft.println("OTHER");
  }
}

void LCD_printBLEstatus()
{
  if(BLE_CON)
  {
    tft.fillRect(137,210,144,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(137,210);
    tft.println("CONNECTED");    
  }
  else
  {
    tft.fillRect(137,210,144,17, ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(137,210);
    tft.println("DISCONNECTED");
  }
}

void LCD_printData(float angleX, float angleXvel)
{
  //MASK OLD DATA
  tft.fillRect(149,87,100,17, ILI9341_BLACK); //MASK ANGLE
  tft.fillRect(197,117,100,17, ILI9341_BLACK); //MASK ANGLE VEL

  //Set Text color
  tft.setTextColor(ILI9341_RED);
  //Printing angle
  int sizeAngle = String(angleX).length();
  tft.setTextSize(2);
  tft.setCursor(149,90);
  tft.println(angleX);

  tft.setTextSize(1);
  tft.setCursor(149+sizeAngle*12+3,87);
  tft.print("o");

  //Printing angle velocity
  int sizeAngleROC = String(angleXvel).length();
  tft.setTextSize(2);
  tft.setCursor(197,120);
  tft.println(angleXvel);

  tft.setTextSize(1);
  tft.setCursor(197+sizeAngleROC*12+3,117);
  tft.print("o");
  tft.setTextSize(2);
  tft.setCursor(197+sizeAngleROC*12+8,120);
  tft.print("/sec");    

  delay(5);
}

