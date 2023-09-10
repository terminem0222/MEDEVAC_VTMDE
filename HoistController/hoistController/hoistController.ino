//Controlling the Hoist

#define SPEED_PIN 19
#define LOWERING_PIN 13
#define RAISING_PIN 14

void config_speed_ctrl_pin()
{
  pinMode(SPEED_PIN, OUTPUT);
}

void config_lowering_pin()
{
  pinMode(LOWERING_PIN, OUTPUT);
}

void config_raising_pin()
{
  pinMode(RAISING_PIN, OUTPUT);
}

void set_pwm_speed(uint8_t duty_cycle)
{
  analogWrite(SPEED_PIN, duty_cycle / 100 * 255);
}

void set_raise_mode()
{
  digitalWrite(LOWERING_PIN, LOW);
  digitalWrite(RAISING_PIN, HIGH);
}

void set_lower_mode()
{
  digitalWrite(RAISING_PIN, LOW);
  digitalWrite(LOWERING_PIN, HIGH);
}

void stop()
{
  digitalWrite(LOWERING_PIN, LOW);
  digitalWrite(RAISING_PIN, LOW);
  analogWrite(SPEED_PIN, 0);
}

void setup() 
{
  // put your setup code here, to run once:

}

void loop() 
{
  // put your main code here, to run repeatedly:

}
