/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-switch
 */

#include <ezButton.h>

ezButton toggleSwitch(5);  // create ezButton object that attach to pin 7;

void setup() {
  Serial.begin(9600);
  toggleSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds
}

void loop() {
  toggleSwitch.loop(); // MUST call the loop() function first

  if (toggleSwitch.isPressed())
  {  
    Serial.println("The switch: OFF -> ON");
    Serial.println("Mode Selection: AUTOMATIC");
  }
  
  if (toggleSwitch.isReleased())
  {
    Serial.println("The switch: ON -> OFF");
    Serial.println("Mode Selection: MANUAL");  
  }      
  /**
  int state = toggleSwitch.getState();
  if (state == HIGH)
    Serial.println("The switch: OFF");
  else
    Serial.println("The switch: ON");
  **/    
}