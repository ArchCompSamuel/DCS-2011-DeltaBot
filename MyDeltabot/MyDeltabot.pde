/*Samuel Whitby
Arch1391
Deltabot
*/

#include <Servo.h> 
 
Servo servo1;
Servo servo2;
Servo servo3;


#define MAXANGLE 90
#define MINANGLE -90

float xPos = 0;
float yPos = 0;
float zPos = -230;

int count = 1;
int potpin = 0;
int flexval;
int flexheight;

char Axis;
int value;

float s1_theta;
float s2_theta;
float s3_theta;
float old_T1;
float old_T2;
float old_T3;
float threshold = 3.0;

int result;

/*
Inverse Kinematics and Forward Kinematics CODE FROM
http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
by mzavatsky
*/

// robot geometry
// (look at pics above for explanation
// mzavatsky
const float e = 40.0;     // end effector
const float f = 130.0;     // base
const float re = 230.5;
const float rf = 60.0;
 
 // trigonometric constants
 // mzavatsky
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;    // PI
const float sin120 = sqrt3/2.0;   
const float cos120 = -0.5;        
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;
 
 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 // mzavatsky
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
 // mzavatsky
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 // mzavatsky
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
     return status;
 }
 


void setup() 
{ 
  xPos = 0;
  yPos = 0;
  zPos = 200;
  servo1.attach(2);
  servo2.attach(4);
  servo3.attach(6);  
  Serial.begin(9600);
}

// Serialprint and Servo Control CODE by prutsers.wordpress.com
void loop()
{
  
  if ( Serial.available()) {
  char ch = Serial.read();
  switch(ch) {
    case 'X':
      Axis='X';
      break;
    case 'Y':
      Axis='Y';
      break;
    case '0' ... '9':
    value = map(ch*10, 300, 600, 10, 100);
      if (Axis=='X')
      {
        xPos += value;
      }
      else if (Axis=='Y')
      {
        yPos += value;
      }
      break;
  }
  }
  
  result = delta_calcInverse(xPos, yPos, zPos, s1_theta, s2_theta, s3_theta);
 
  Serial.print(result == 0 ? "ok" : "no");
  char buf[10];
  dtostrf(xPos, 4, 0, buf);
  Serial.print(" X");
  Serial.print(buf);
  dtostrf(yPos, 4, 0, buf);
  Serial.print(" Y");
  Serial.print(buf);
  dtostrf(zPos, 4, 0, buf);
  Serial.print(" Z");
  Serial.print(buf);
 
  dtostrf(s1_theta, 6, 2, buf);
  Serial.print(" T1");
  Serial.print(buf);
  dtostrf(s2_theta, 6, 2, buf);
  Serial.print(" T2");
  Serial.print(buf);
  dtostrf(s3_theta, 6, 2, buf);
  Serial.print(" T3");
  Serial.print(buf);
 
  Serial.println("");
  
  if (result == 0) {
    if (abs(old_T1 - s1_theta) > threshold || abs(old_T2 - s2_theta) > threshold || abs(old_T3 - s3_theta) > threshold)
    {
      servo1.write(s1_theta+90.0); 
      old_T1 = s1_theta;
      servo2.write(s2_theta+90.0);
      old_T2 = s2_theta;
      servo3.write(s3_theta+90.0);
      old_T3 = s3_theta;
    }
  delay(250);
  }
  delay(250);
}
