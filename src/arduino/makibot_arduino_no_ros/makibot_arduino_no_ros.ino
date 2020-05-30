//Makibot Test
/*Data Format
   LeftMotor Speed, RightMotor Speed, Headlight Intensity, Picker(bool), AUX1, AUX2
    255,255,50,1,0,0%

  PWM = T per Rotation
  50 = 5.4
  100 = 3
  150 = 2.1
  200 = 1.7
  250 = 1.3
*/

#include <Servo.h>

Servo carrier_servo;  // create servo object to control a servo
#define motorR_A 9
#define motorR_B 6
#define motorL_A 5
#define motorL_B 3

#define TRIG_PIN A0
#define ECHO_PIN A1

#define LED1 10
#define LED2 11
#define LED3 12

#define BUZZ 13


#define forward false
#define reverse true

#define left true
#define right false

String rec;

void setup()
{

  Serial.begin(9600);
  //  pinMode(motorR_A, OUTPUT);
  //  pinMode(motorR_B, OUTPUT);
  //  pinMode(motorL_A, OUTPUT);
  //  pinMode(motorL_B, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  //  pinMode(LED1, OUTPUT);
  //  pinMode(LED2, OUTPUT);
  //  pinMode(LED3, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  carrier_servo.attach(11);  // attaches the servo on pin 9 to the servo object

}


const int rest_pos = 70;
const int drop_pos = 15;
int servo_pos = rest_pos; //130 rest, 90 drop

void loop()
{


  if (Serial.available())
  {
    // m,dirL, spdL, dirR,  spdR, servo_pos, buzz_state
    // m,0,0,0,0,140,1
    // m,0,0,0,250,140,1
    // m,0,250,0,0,140,1
    // m,0,250,0,250,140,1
    // m,1,250,1,250,140,1
    if (Serial.read() == 'm')
    {
      bool dir_L = Serial.parseInt();
      int spd_L = Serial.parseInt();
      bool dir_R = Serial.parseInt();
      int spd_R = Serial.parseInt();
      int servo_state = Serial.parseInt();
      int buzz_state = Serial.parseInt();

      rotate(motorL_A, motorL_B, spd_L, dir_L);
      rotate(motorR_A, motorR_B, spd_R, dir_R);

      if ( servo_state&& (servo_pos == rest_pos))
      {
        for (int i = servo_pos; i > drop_pos; i--)
        {
          carrier_servo.write(i);
          delay(20);
        }
        servo_pos = drop_pos;
      }

      else if( (servo_state == 0) && (servo_pos == drop_pos))
      {
        for (int i = servo_pos; i < rest_pos; i++)
        {
          carrier_servo.write(i);
          delay(20);
        }
        servo_pos = rest_pos;
      }



    }

  }

  /*

    if (getDistance() < 10)
    {
      Backward(spd);
      delay(1000);
      brake();
    }
    if (Serial.available() > 0)
    {
      char rec = Serial.read();
      if (rec == 'F')
        Forward(spd, spd);
      else if (rec == 'B')
        Backward(spd);
      else if (rec == 'R')
        Right(spd);
      else if (rec == 'L')
        Left(spd);
      else if (rec == 'I')
        turnRight(spd);
      else if (rec == 'G')
        turnLeft(spd);
      else if (rec == 'S')
        brake();
      else if (rec == 'L')
        turnLeft(spd);
      else if (rec == 'W')
        digitalWrite(LED1, HIGH);
      else if (rec == 'w')
        digitalWrite(LED1, LOW);
      else if (rec == 'U')
        digitalWrite(LED2, HIGH);
      else if (rec == 'u')
        digitalWrite(LED2, LOW);
      else if (rec == 'V')
        digitalWrite(BUZZ, HIGH);
      else if (rec == 'v')
        digitalWrite(BUZZ, LOW);
      else if ( rec == 'z')
        buzz(5);
    }
  */

}

void rotate(uint8_t motorA, uint8_t motorB, uint16_t sp, bool dir)
{
  if (dir)
  {
    digitalWrite(motorA, 255);
    analogWrite(motorB, 0);
  }
  else if (!dir)
  {
    analogWrite(motorA, 0);
    analogWrite(motorB, sp);
  }
}

void turnLeft(uint16_t sp)
{
  rotate(motorR_A, motorR_B, sp, forward);
  rotate(motorL_A, motorL_B, 0, forward);
}

void turnRight(uint16_t sp)
{
  rotate(motorR_A, motorR_B, 0, forward);
  rotate(motorL_A, motorL_B, sp, forward);
}

void Left(uint16_t sp)
{
  rotate(motorR_A, motorR_B, sp, forward);
  rotate(motorL_A, motorL_B, sp, reverse);
}

void Right(uint16_t sp)
{
  rotate(motorR_A, motorR_B, sp, reverse);
  rotate(motorL_A, motorL_B, sp, forward);
}

void brake()
{
  rotate(motorR_A, motorR_B, 0, forward);
  rotate(motorL_A, motorL_B, 0, forward);
}

void Backward(uint16_t sp)
{
  rotate(motorR_A, motorR_B, sp, reverse);
  rotate(motorL_A, motorL_B, sp, reverse);
}

void Forward(uint16_t spL, uint16_t spR)
{
  rotate(motorR_A, motorR_B, spR, forward);
  rotate(motorL_A, motorL_B, spL, forward);
}

void buzz(uint16_t duration)
{
  duration = duration / 0.1;
  for (int i = 0; i < duration; i++)
  {
    digitalWrite(BUZZ, HIGH);
    delay(50);
    digitalWrite(BUZZ, LOW);
    delay(50);
  }
}

float getDistance()
{
  float duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the TRIG_PIN on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the ECHO_PIN, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  return distance;
}
