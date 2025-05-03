
#include <Servo.h>
Servo steeringServo;

// right rear motor direction and speed control
int pinENA = 6;
int pinIN1 = 7;
int pinIN2 = 8;


// left rear motor direction and speed control
int pinENB = 3;
int pinIN3 = 4;
int pinIN4 = 5;


// steering servo motor
int pos =  0;
int pinSTR = 9;

void setup(){
 
//right motor setup
 pinMode(pinIN1, OUTPUT);
 pinMode(pinIN2, OUTPUT);
 pinMode(pinENA, OUTPUT);
 
 //left motor setup
 pinMode(pinIN3, OUTPUT);
 pinMode(pinIN4, OUTPUT);
 pinMode(pinENB, OUTPUT);

  // steering setupa
 pinMode(pinSTR, OUTPUT);
 steeringServo.attach(pinSTR);
 
 
}

void loop(){
 
  //right rear motor forward
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2,LOW);
  analogWrite(pinENA,128);
  delay(3000);
 
  //right rear motor stop
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2,LOW);
  analogWrite(pinENA,0);
  delay(3000);
 
  //left rear motor forward
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4,LOW);
  analogWrite(pinENB,128);
  delay(3000);
 
  //left rear motor stop
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4,LOW);
  analogWrite(pinENB,0);
  delay(3000);
 
  //right rear motor reverse
  digitalWrite(pinIN1,LOW);
  digitalWrite(pinIN2,HIGH);
  analogWrite(pinENA,128);
  delay(3000);
 
  //right rear motor stop
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2,LOW);
  analogWrite(pinENA,0);
  delay(3000);
 
  //left rear motor backwards
  digitalWrite(pinIN3,LOW);
  digitalWrite(pinIN4,HIGH);
  analogWrite(pinENB,128);
  delay(3000);
 
  //left read motor stop
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4,LOW);
  analogWrite(pinENB,0);
  delay(3000);

// trun right
 for (pos=90; pos <180; pos+=1){
   steeringServo.write(pos);
   delay(15);
 }
 
 //trun left
 for (pos=180; pos <0; pos-=1){
   steeringServo.write(pos);
   delay(15);
 }
 
 //recenter steering angle
 for (pos=0; pos <90; pos+=1){
  steeringServo.write(pos);
   delay(15);
 }
 
}

