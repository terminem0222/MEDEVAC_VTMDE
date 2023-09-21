/*
  SD card datalogger
 
 This example shows how to log data from three analog sensors
 to an SD card using the SD library.
 	
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4,  pin 10 on Teensy with audio board
 
 */

#include <SD.h>
#include <SPI.h>
#define TEST_MODE 1

uint8_t counter = 0;
long long sample = 0;
uint8_t cold_boot = 0;
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
const int chipSelect = BUILTIN_SDCARD;
void setup_SD()
{
  //Comment out for integration/////////////////////////////
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  ///////////////////////////////////////////////////////////

  Serial.print("Initializing SD card...");


  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");
  cold_boot = 0;
}

void setup()
{
  setup_SD();

}

void writeToSD()
{
  String dataString = "Test ";
  
  int randomNum = random(-300, 300);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (TEST_MODE == 1)
  {
    if (cold_boot == 0)
    {
      dataString = "Test Start ";
      cold_boot = 1;
        // if the file is available, write to it:
      if (dataFile) 
      {
        dataFile.println(dataString);
        dataFile.close();
        // print to the serial port too:
        Serial.println(dataString);
      } 
      else 
      {
        // if the file isn't open, pop up an error:
        Serial.println("error opening datalog.txt");
      }
      delay(100); // run at a reasonable not-too-fast speed
    }
    else 
    {
      if(dataFile)
      {
        dataFile.print("Sample: ");
        dataFile.print(String(sample));
        dataFile.print(" ");
        dataFile.print("Angle Velocity: ");
        dataFile.print(String(randomNum));
        dataFile.print(" ");
        dataFile.print("Angle: ");
        randomNum = randomNum + 10 - 6 *2 / 3;
        dataFile.print(String(randomNum));
        dataFile.print(" ");
        dataFile.println(" ");
        dataFile.close();
        Serial.print("Sample: ");
        Serial.print(String(sample));
        Serial.print("Angle Velocity: ");
        Serial.print(String(randomNum));
        Serial.print("Angle: ");
        Serial.print(String(randomNum + 10 - 6 * 2/ 3));
        Serial.println(" ");
        sample++;
      }
      else 
      {
        // if the file isn't open, pop up an error:
        Serial.println("error opening datalog.txt");
      }
      delay(100); // run at a reasonable not-too-fast speed
      
    }
  }
  // make a string for assembling the data to log:
  
  //int sensor = analogRead(analogPin);
  //dataString += String(sensor);


  
  else 
  {
    if (dataFile) 
    {
      dataString = "TEST";

      dataFile.print(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    } 
    else 
    {
      // if the file isn't open, pop up an error:
      Serial.println("error opening datalog.txt");
    }
    delay(100); // run at a reasonable not-too-fast speed
  } 
}
void loop()
{
  
  if (counter < 1)
  {
    writeToSD();
  }
  else {
  {
    Serial.println("finished writing 1 test to SD");
  }
  }
}





