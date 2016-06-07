// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 
 
Servo myservo1;  // create servo object to control a servo 
Servo myservo2;  // create servo object to control a servo 
 
void setup() 
{ 
  Serial.begin(115200);
  Serial2.begin(115200);
 // while(!Serial);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(13,OUTPUT);
  myservo1.attach(9);  // attaches the servo on pin 9 to the servo1 object 
  myservo2.attach(10);  // attaches the servo on pin 10 to the servo2 object 
  myservo1.write(0);
  myservo2.write(90);
  delay(100); 
} 

void  reverseMotor(int index){
  static int clockwise1 = 0, clockwise2 = 1;  
  if(index == 0){
    if(clockwise1){
      digitalWrite(4,HIGH);
    }else{
      digitalWrite(4,LOW);
    }
    clockwise1 = !clockwise1;
  }else{
    if(clockwise2){
      digitalWrite(5,HIGH);
    }else{
      digitalWrite(5,LOW);
    }
    clockwise2 = !clockwise2;  
  }
}

void cb(void)
{
  static int count=0;
  static int clockwise1=1,clockwise2=1;
  static int phase1=0,phase2=90;
  if(clockwise1)
    myservo1.write(++phase1);
  else
    myservo1.write(--phase1);
    
  if(clockwise2)
    myservo2.write(++phase2);
  else
    myservo2.write(--phase2);
    
  if(phase1 == 180)
     clockwise1 = 0;
  else if(phase1 == 0)
     clockwise1 = 1;
  
  if(phase2 == 180)
     clockwise2 = 0;
  else if(phase2 == 0)
     clockwise2 = 1;
  if(++count == 200){
    static int reverse=0;
    count = 0;
    if(reverse == 0){
      reverseMotor(0);  
    }else{
      reverseMotor(1);
    }
    reverse = !reverse;
  }
}
void loop() 
{
  static int count=0;
  delay(10);
  cb();
  count++;
  Serial.println(count);
  Serial2.println(count);
} 
