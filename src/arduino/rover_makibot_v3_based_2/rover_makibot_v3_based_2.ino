/*Data Format
   LeftMotor Speed, RightMotor Speed, Headlight Intensity, Picker(bool), AUX1, AUX2
    255,255,50,1,0,0%
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>

#define pwm_pinL 9
#define dir_pinL 17
#define pwm_pinR 11
#define dir_pinR 23
#define pwm_pinL2 8
#define dir_pinL2 16
#define pwm_pinR2 10
#define dir_pinR2 22

#define front_tof_pin A0
#define rear_tof_pin A1

#define MAX_SPEED 24
#define LOOPTIME 100

int t2 = 0;
int t1;
int period;

volatile int pulseL = 0;
volatile int pulseR = 0;
volatile int pulseL2 = 0;
volatile int pulseR2 = 0;
double rpmL;
double rpmR;
double rpmL2;
double rpmR2;
double curr_spdL; // cm/s
double des_spdL; // cm/s
double curr_spdR; // cm/s
double des_spdR; // cm/s
double curr_spdL2; // cm/s
double des_spdL2; // cm/s
double curr_spdR2; // cm/s
double des_spdR2; // cm/s
int16_t pwmL = 0;
int16_t pwmR = 0;
int16_t pwmL2 = 0;
int16_t pwmR2 = 0;

double gen_speed = 15;

void isrL() //interrupt service routine
{
  pulseL++;
}

void isrR() //interrupt service routine
{
  pulseR++;
}

void isrL2() //interrupt service routine
{
  pulseL2++;
}

void isrR2() //interrupt service routine
{
  pulseR2++;
}

ros::NodeHandle  nh;
String rec;

void messageCb( const geometry_msgs::Twist& cmd)
{

  float spd = cmd.linear.x;
  float ang = cmd.angular.z;

  if (spd > 0)
  {
    des_spdL = gen_speed;
    des_spdR = gen_speed;
    des_spdL2 = gen_speed;
    des_spdR2 = gen_speed;
    digitalWrite(dir_pinL, HIGH);
    digitalWrite(dir_pinR, LOW);
    digitalWrite(dir_pinL2, HIGH);
    digitalWrite(dir_pinR2, LOW);
  }
  else if (spd < 0)
  {
    des_spdL = gen_speed;
    des_spdR = gen_speed;
    des_spdL2 = gen_speed;
    des_spdR2 = gen_speed;
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, HIGH);
    digitalWrite(dir_pinL2, LOW);
    digitalWrite(dir_pinR2, HIGH);
  }
//  else if (ang > 0 )
//  {
//    des_spdL = 0;
//    des_spdR = gen_speed;
//    digitalWrite(dir_pinL, HIGH);
//    digitalWrite(dir_pinR, LOW);
//  }
//  else if ( ang < 0 )
//  {
//    des_spdL = gen_speed;
//    des_spdR = 0;
//    digitalWrite(dir_pinL, HIGH);
//    digitalWrite(dir_pinR, LOW);
//  }
  else if (ang > 0 )
  {
    des_spdL = gen_speed;
    des_spdR = gen_speed;
    des_spdL2 = gen_speed;
    des_spdR2 = gen_speed;
    digitalWrite(dir_pinL, LOW);
    digitalWrite(dir_pinR, LOW);
    digitalWrite(dir_pinL2, LOW);
    digitalWrite(dir_pinR2, LOW);
  }
  else if ( ang < 0 )
  {
    des_spdL = gen_speed;
    des_spdR = gen_speed;
    des_spdL2 = gen_speed;
    des_spdR2 = gen_speed;
    digitalWrite(dir_pinL, HIGH);
    digitalWrite(dir_pinR, HIGH);
    digitalWrite(dir_pinL2, HIGH);
    digitalWrite(dir_pinR2, HIGH);
  }
  else if (spd == 0)
  {
    des_spdL = 0;
    des_spdR = 0;
    des_spdL2 = 0;
    des_spdR2 = 0;
  }
  //  else if (String(received_command) == "ROT_LEFT")
  //  {
  //    des_spdL = gen_speed;
  //    des_spdR = gen_speed;
  //    digitalWrite(dir_pinL, LOW);
  //    digitalWrite(dir_pinR, LOW);
  //  }
  //  else if (String(received_command) == "ROT_RIGHT")
  //  {
  //    des_spdL = gen_speed;
  //    des_spdR = gen_speed;
  //    digitalWrite(dir_pinL, HIGH);
  //    digitalWrite(dir_pinR, HIGH);
  //  }
  //  else if (String(received_command) == "FRONT_PROXIMITY_Q")
  //  {
  //    Serial.println(front_tof);
  //  }
  //  else if (String(received_command) == "REAR_PROXIMITY_Q")
  //  {
  //    Serial.println(rear_tof);
  //  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &messageCb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);

  attachInterrupt(4, isrL, RISING);
  attachInterrupt(2, isrR, RISING);
  attachInterrupt(5, isrL2, RISING);
  attachInterrupt(3, isrR2, RISING); 

  pinMode(pwm_pinL, OUTPUT);
  pinMode(dir_pinL, OUTPUT);
  pinMode(pwm_pinR, OUTPUT);
  pinMode(dir_pinR, OUTPUT);
  pinMode(pwm_pinL2, OUTPUT);
  pinMode(dir_pinL2, OUTPUT);
  pinMode(pwm_pinR2, OUTPUT);
  pinMode(dir_pinR2, OUTPUT);
  pinMode(front_tof_pin, INPUT);
  pinMode(rear_tof_pin, INPUT);

  analogWrite(pwm_pinR, 0);
  analogWrite(pwm_pinL, 0);
  analogWrite(pwm_pinR2, 0);
  analogWrite(pwm_pinL2, 0);
  digitalWrite(dir_pinL, HIGH);
  digitalWrite(dir_pinR, LOW);
  digitalWrite(dir_pinL2, HIGH);
  digitalWrite(dir_pinR2, LOW);
}

int ctr = 0;

void loop()
{
  detachInterrupt(4);           //detaches the interrupt pin 19
  detachInterrupt(2);           //detaches the interrupt pin 21
  detachInterrupt(5);           //detaches the interrupt pin 18
  detachInterrupt(3);           //detaches the interrupt pin 20

  t1 = millis();    //finds the time
  bool front_tof = digitalRead(front_tof_pin);
  bool rear_tof = digitalRead(rear_tof_pin);

  period = t1 - t2;
  if (period == 100)
  {

    float lp = pulseL;
    float rp = pulseR;
    float lp2 = pulseL2;
    float rp2 = pulseR2;
    
    rpmL = (lp / 4.3) * 60;
    rpmR = (rp / 4.3) * 60;
    rpmL2 = (lp2 / 4.3) * 60;
    rpmR2 = (rp2 / 4.3) * 60;

    curr_spdL = (rpmL / 60) * (2 * PI * 0.08) * 100;
    curr_spdR = (rpmR / 60) * (2 * PI * 0.08) * 100;
    curr_spdL2 = (rpmL2 / 60) * (2 * PI * 0.08) * 100;
    curr_spdR2 = (rpmR2 / 60) * (2 * PI * 0.08) * 100;

    pulseL = 0;
    pulseR = 0;
    pulseL2 = 0;
    pulseR2 = 0;

    t2 = t1;           //saves the current time
    int errL = des_spdL - curr_spdL;
    pwmL = pwmL + (errL * 0.2); // proportional

    int errR = des_spdR - curr_spdR;
    pwmR = pwmR + (errR * 0.2); // proportional

    int errL2 = des_spdL2 - curr_spdL2;
    pwmL2 = pwmL2 + (errL2 * 0.2); // proportional

    int errR2 = des_spdR2 - curr_spdR2;
    pwmR2 = pwmR2 + (errR2 * 0.2); // proportional

    pwmL = (pwmL > 255) ? 255 : pwmL;
    pwmR = (pwmR > 255) ? 255 : pwmR;
    pwmL2 = (pwmL2 > 255) ? 255 : pwmL2;
    pwmR2 = (pwmR2 > 255) ? 255 : pwmR2;
    
    pwmL = (pwmL < 0) ? 0 : pwmL;
    pwmR = (pwmR < 0) ? 0 : pwmR;
    pwmL2 = (pwmL2 < 0) ? 0 : pwmL2;
    pwmR2 = (pwmR2 < 0) ? 0 : pwmR2;

    analogWrite(pwm_pinL, pwmL);      //sets the desired speed
    analogWrite(pwm_pinR, pwmR);      //sets the desired speed
    analogWrite(pwm_pinL2, pwmL2);      //sets the desired speed
    analogWrite(pwm_pinR2, pwmR2);      //sets the desired speed
    publishSpeed(LOOPTIME);
  }

  attachInterrupt(4, isrL, RISING);
  attachInterrupt(2, isrR, RISING);
  attachInterrupt(5, isrL2, RISING);
  attachInterrupt(3, isrR2, RISING);

  nh.spinOnce();


}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = curr_spdL;    //left wheel speed (in m/s)
  speed_msg.vector.y = curr_spdR;   //right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  //  nh.loginfo("Publishing odometry");
}
