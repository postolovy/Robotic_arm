#include <Servo.h> 

Servo servoBase; 
Servo servoArm; 
Servo servoClaw; 


#define dirPin 3
#define stepPin 5
#define servoBasePin 9
#define servoArmPin 10
#define servoClawPin 11


void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  //Servo motors attached pins
  servoBase.attach(9); 
  servoArm.attach(10); 
  servoClaw.attach(11); 
  servoBase.write(80);
  servoArm.write(100);
  servoClaw.write(100); 
}
void loop() {
  //SERVO 3
  moveServo(0, 100, servoClaw);
  delay(1000); 

  //SERVO 1 
  moveServo(80, 157, servoBase); 
  delay(1000);

  //SERVO2
  moveServo(100, 44, servoArm); 
  delay(1000); 

  //SERVO 3
  moveServo(100, 0, servoClaw);
  delay(2000); 



  //SERVO 1 
  moveServo(157, 80, servoBase); 
  delay(1000);

  //SERVO2
  moveServo(44, 100, servoArm); 
  delay(2000); 


  //STEPPER 
  digitalWrite(dirPin,LOW); 
  moveStepper(100); 
  delay(1000);


  // //STEPPER 
  // digitalWrite(dirPin,HIGH); 
  // moveStepper(110); 
  // delay(1000);



  //SERVO 1 
  moveServo(80, 157, servoBase); 
  delay(1000);

  //SERVO2
  moveServo(100, 44, servoArm); 
  delay(1000); 

  //SERVO 3
  moveServo(0, 100, servoClaw);
  delay(2000);




  //SERVO 1 
  moveServo(157, 80, servoBase); 
  delay(1000);

  //SERVO2
  moveServo(44, 100, servoArm); 
  delay(1000); 

  //SERVO 3
  moveServo(100, 0, servoClaw);
  delay(1000);

  //STEPPER 
  digitalWrite(dirPin,HIGH); 
  moveStepper(100); 
  delay(10000);



  // //STEPPER 
  // digitalWrite(dirPin,LOW); 
  // moveStepper(100); 
  // delay(1000);

  // //STEPPER 
  // digitalWrite(dirPin,HIGH); 
  // moveStepper(100); 
  // delay(1000);
}

void moveStepper(int steps){  //dir - HIGH, LOW to change direction 
  for(int i = 0; i < steps; i++){ 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(4500); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(4500);     
  }
}

void moveServo(int initPos, int finalPos, Servo &servoName){ //move servo with initial and final positions 
  if(initPos < finalPos){
    for(int i = initPos; i < finalPos; i++){ 
      servoName.write(i); 
      delay(10); 
    }
  }
  else{ 
    for(int i = initPos; i > finalPos; i--){ 
      servoName.write(i); 
      delay(10); 
    }
  }
}



 
