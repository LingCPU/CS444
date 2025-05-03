#include <Servo.h>

// vehicle constants
double L = 23.50; // cm
double W = 16.25; // cm
double aspectRatio = W/L;
double deg2Rad = M_PI/180.0;
double maxSpeed = 128;
double minSteerAngle =10;
double maxSteerAngle =170;

int hiLow1 =0;
int hiLow2 =0;
int hiLow3 =0;
int hiLow4 =0;

//create servo for steering
Servo steeringServo;
int steeringPos = 90;

// create speed variable
int speed = 0;
int leftSpeed;
int rightSpeed;

// setup for serial comm
String cmdRec = "";
String cmd = "";
String value = "";
int intValue=0;
int portSpeed =9600;

// set pin numbers

//right motor
  const int pinENA = 3;
  const int pinIN1 = 4;    
  const int pinIN2 = 5;

//steering
  const int pinSTR = 9;

//left motor
  const int pinENB = 6;
  const int pinIN3 = 7;    
  const int pinIN4 = 8;

void setup()
{
// serial port setup
   Serial.begin(portSpeed);
   
// steering setup
   pinMode(pinSTR, OUTPUT);
   steeringServo.attach(pinSTR);
 
// Right motor setup
   pinMode(pinENA, OUTPUT);
   pinMode(pinIN1, OUTPUT);
   pinMode(pinIN2, OUTPUT);
 
  //Left motor setup
   pinMode(pinENB, OUTPUT);
   pinMode(pinIN3, OUTPUT);
   pinMode(pinIN4, OUTPUT);
}

void loop()
{  
   
 //read serial port
 if(Serial.available()) {    
     
       cmdRec= Serial.readStringUntil('\n');
       cmd=cmdRec.substring(0,3);
       value=cmdRec.substring(3,6);
       intValue = value.toInt();
   
       //Serial.print("Recieved ");
       //Serial.println(cmd);
       //Serial.print("and ");
       //Serial.println(value);
  }
 
 if(cmd == "STR")
   {
   
   if(intValue == 90)
   {
     steeringPos = 90;
     leftSpeed = speed;
     rightSpeed = speed;
   }
   else
   {
     // steering limits in degrees
     if(intValue<minSteerAngle) intValue = minSteerAngle;
     if(intValue>maxSteerAngle) intValue = maxSteerAngle;
     steeringPos = intValue;
   }
   
  }
 
  if(cmd == "FWD")
  {
    steeringPos = 90;
    speed = intValue;
    
    //limit speed
    if(speed < 0) speed =0;
    if(speed > maxSpeed) speed=maxSpeed;
    
    leftSpeed  = speed;
    rightSpeed = speed;
    
    // set the pin polarity
    hiLow1 =1;
    hiLow2 =0;
    hiLow3 =1;
    hiLow4 =0;
  }
 
  if(cmd == "BKW")
  {
    steeringPos=90;
    speed = intValue;
    
     //limit speed
    if(speed < 0) speed =0;
    if(speed > maxSpeed) speed=maxSpeed;
    
    leftSpeed  = speed;
    rightSpeed = speed;
    
    //set polarity
    hiLow1 =0;
    hiLow2 =1;
    hiLow3 =0;
    hiLow4 =1;
  }
 
  if(cmd == "STP")
  {
   speed =0;
   leftSpeed = 0;
   rightSpeed = 0;
   steeringPos = 90;
   hiLow1 = 0;
   hiLow2 = 0;
   hiLow3 = 0;
   hiLow4 = 0;  
  }
  
   
  // send steering command
    steeringServo.write(steeringPos);

   //send right rear motor command
    digitalWrite(pinIN1,hiLow1);
    digitalWrite(pinIN2,hiLow2);
    analogWrite(pinENA,rightSpeed);
 
   
    //send left rear motor command
    digitalWrite(pinIN3, hiLow3);
    digitalWrite(pinIN4,hiLow4);
    analogWrite(pinENB,leftSpeed);

}
