 
 #include <Servo.h>

 Servo steeringServo;
 int steeringPos = 0;    // variable to store the servo position 
 int pinSTR = 9;
 
String cmdRec = "";
String cmd = "";
String value = "";
int portSpeed =9600;




void setup() {
  
  // seria port setup
    Serial.begin(portSpeed);
    
  // steering setup
    pinMode(pinSTR, OUTPUT);
    steeringServo.attach(pinSTR);  // attaches the servo on pin 9 to the servo object
    
    
}

void loop() {
  
//read serial port
    if(Serial.available()) {    
     
       cmdRec= Serial.readStringUntil('\n');
       cmd=cmdRec.substring(0,3);
       value=cmdRec.substring(3,6);
       
       //Serial.print("Recieved ");
       Serial.println(cmd);
       //Serial.print("and ");
       Serial.println(value);
    }

// issue steering command    
        if(cmd == "STR")
        {
          steeringPos = value.toInt();
          steeringServo.write(steeringPos);
          delay(1000);
          //straighten out
          steeringPos = 90;
         steeringServo.write(steeringPos);
          cmd="";
          value="";  
        }
    
    
}

