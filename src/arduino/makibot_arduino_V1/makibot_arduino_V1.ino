//Makibot Test
/*Data Format
   LeftMotor Speed, RightMotor Speed, Headlight Intensity, Picker(bool), AUX1, AUX2
    255,255,50,1,0,0%
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>

Servo claw_servo;  // create servo object to control a servo
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

uint16_t spd = 255;

ros::NodeHandle  nh;
String rec;
void messageCb( const std_msgs::String& cmd)
{
  if (cmd.data)
  {
    rec = String(cmd.data);
    if (rec == "F")
      Forward(spd, spd);
    else if (rec == "B")
      Backward(spd);
    else if (rec == "R")
      Right(spd);
    else if (rec == "L")
      Left(spd);
    else if (rec == "I")
      turnRight(spd);
    else if (rec == "G")
      turnLeft(spd);
    else if (rec == "S")
      brake();
    else if (rec == "W")
      claw_servo.write(80);
    else if (rec == "w")
      claw_servo.write(130);
    else if (rec == "U")
      digitalWrite(LED2, HIGH);
    else if (rec == "u")
      digitalWrite(LED2, LOW);
    else if (rec == "V")
      digitalWrite(BUZZ, HIGH);
    else if (rec == "v")
      digitalWrite(BUZZ, LOW);
    else if ( rec == "z")
      buzz(5); 
  }
  delay(100);
  brake();
}

ros::Subscriber<std_msgs::String> sub("cmd", &messageCb );

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  //Serial.begin(9600);
  pinMode(motorR_A, OUTPUT);
  pinMode(motorR_B, OUTPUT);
  pinMode(motorL_A, OUTPUT);
  pinMode(motorL_B, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUZZ, OUTPUT);  
  claw_servo.attach(10);  // attaches the servo on pin 9 to the servo object
  
}

String prev_rec;
int ctr = 0;
void loop()
{
/*
  if(prev_rec == rec)
  {
    ctr++;
    prev_rec = rec;
    if(ctr > 20)
     brake();
  } */
  
  
  nh.spinOnce();
  delay(1);


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
    analogWrite(motorA, sp);
    digitalWrite(motorB, LOW);
  }
  else
  {
    analogWrite(motorA, LOW);
    digitalWrite(motorB, sp);
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
  for (int i = 0; i < duration; i++)
    digitalWrite(BUZZ, HIGH);
  delay(50);
  digitalWrite(BUZZ, LOW);
  delay(50);
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
