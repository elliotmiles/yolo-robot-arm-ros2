#include <AccelStepper.h>
#include <Wire.h>

// Base
#define A_DirPin 4
#define A_StepPin 5
#define B_DirPin 6
#define B_StepPin 7
#define C_DirPin 8
#define C_StepPin 9
#define D_DirPin 10
#define D_StepPin 11

//limit swtich pins
#define limit_A A0
#define limit_B A1
#define limit_C A2
#define limit_D A3

#define motorInterfaceType 1



AccelStepper motor_A(motorInterfaceType, A_StepPin, A_DirPin);
AccelStepper motor_B(motorInterfaceType, B_StepPin, B_DirPin);
AccelStepper motor_C(motorInterfaceType, C_StepPin, C_DirPin);
AccelStepper motor_D(motorInterfaceType, D_StepPin, D_DirPin);

// ----- motor X steps per rev: steps per rev * (gear ratio) * microstepping * gearbox(if applicable) -----
long motor_A_steps_rev = 200 * (100/20) * 16;
long motor_B_steps_rev = 200 * (120/60) * 10 * 32;
long motor_C_steps_rev = 200 * (95.0/20.0) * 32;
long motor_D_steps_rev = 200 * (70.0/20.0) * 16;
// ----------------------------------------------------

long reverseDist = 200;

long A_eighthTurn = 0.125 * 200 * (100/20) * 16;
long A_quarterTurn = 0.25 * 200 * (100/20) * 16;
long A_halfTurn = 0.5 * 200 * (100/20) * 16;

long B_eighthTurn = 0.125 * 200 * (120/60) * 10 * 32;
long B_quarterTurn = 0.25 * 200 * (120/60) * 10 * 32;
long B_halfTurn =  0.5 * 200 * (120/60) * 10 * 32;

long C_eighthTurn = 0.125 * 200 * (95/20) * 32;
long C_quarterTurn = 0.25 * 200 * (95/20) * 32;
long C_halfTurn =  0.5 * 200 * (95/20) * 32;

long D_eighthTurn = 0.125 * 200 * (70/20) * 16;
long D_quarterTurn = 0.25 * 200 * (70/20) * 16;
long D_halfTurn = 0.5 * 200 * (70/20) * 16;

void setup() {

  // lim switches
  pinMode(limit_A, INPUT_PULLUP); 
  pinMode(limit_B, INPUT_PULLUP); 
  pinMode(limit_C, INPUT_PULLUP);  
  pinMode(limit_D, INPUT_PULLUP); 

  //----- A HOMING -----
  motor_A.setMaxSpeed(1000);
  motor_A.setAcceleration(200);

  //fast approach
  motor_A.setSpeed(200);
  while (digitalRead(limit_A) == LOW) {
    motor_A.runSpeed();
  }

  delay(200);

  //reverse
  motor_A.setMaxSpeed(200);
  motor_A.setAcceleration(200);
  motor_A.move(-reverseDist);
  while (motor_A.isRunning()) {
    motor_A.run();
  }

  //slow approach
  motor_A.setSpeed(100);
  while (digitalRead(limit_A) == LOW) {
    motor_A.runSpeed();
  }
  motor_A.setCurrentPosition(0);
  // --------------------------------


  //----- D HOMING -----
  motor_D.setMaxSpeed(1000);
  motor_D.setAcceleration(200);

  //fast approach
  motor_D.setSpeed(200);
  while (digitalRead(limit_D) == LOW) {
    motor_D.runSpeed();
  }

  delay(200);

  //reverse
  motor_D.setMaxSpeed(200);
  motor_D.setAcceleration(200);
  motor_D.move(-reverseDist);
  while (motor_D.isRunning()) {
    motor_D.run();
  }

  //slow approach
  motor_D.setSpeed(100);
  while (digitalRead(limit_D) == LOW) {
    motor_D.runSpeed();
  }
  motor_D.setCurrentPosition(0);
  // --------------------------------

  //----- C HOMING -----
  motor_C.setMaxSpeed(1000);
  motor_C.setAcceleration(200);

  //fast approach
  motor_C.setSpeed(-200);
  while (digitalRead(limit_C) == LOW) {
    motor_C.runSpeed();
  }

  delay(200);

  //reverse
  motor_C.setMaxSpeed(200);
  motor_C.setAcceleration(200);
  motor_C.move(reverseDist);
  while (motor_C.isRunning()) {
    motor_C.run();
  }

  //slow approach
  motor_C.setSpeed(-100);
  while (digitalRead(limit_C) == LOW) {
    motor_C.runSpeed();
  }
  motor_C.setCurrentPosition(0);
  // --------------------------------

  //----- B HOMING -----
  motor_B.setMaxSpeed(1000);
  motor_B.setAcceleration(200);

  //fast approach
  motor_B.setSpeed(-1000);
  while (digitalRead(limit_B) == LOW) {
    motor_B.runSpeed();
  }

  delay(200);

  //reverse
  motor_B.setMaxSpeed(2000);
  motor_B.setAcceleration(200);
  motor_B.move(reverseDist * 5);
  while (motor_B.isRunning()) {
    motor_B.run();
  }

  //slow approach
  motor_B.setSpeed(-100);
  while (digitalRead(limit_B) == LOW) {
    motor_B.runSpeed();
  }
  motor_B.setCurrentPosition(0);  
  // --------------------------------
  

  motor_A.setMaxSpeed(600);
  motor_B.setMaxSpeed(6000);  
  motor_C.setMaxSpeed(600);
  motor_D.setMaxSpeed(600);

  // I2C master config
  Wire.begin();

  // serial setup
  Serial.begin(115200);
  Serial.println("Homing finished");


  delay(500);

  Serial.println("Main loop started");

}



void pickup() {
  // send on channel 8
  Wire.beginTransmission(8);
  // turn pump ON and valve CLOSED (picking up card)
  Wire.write(2);             
  Wire.endTransmission();
  delay(1000); 

  // send on channel 8
  Wire.beginTransmission(8);
  // turn pump OFF and valve CLOSED (holding card for movement)
  Wire.write(1);             
  Wire.endTransmission();
  delay(1000); 
}

void drop() {
  // send on channel 8
  Wire.beginTransmission(8);
  // turn pump OFF and valve OPEN (drop card)
  Wire.write(3);             
  Wire.endTransmission();
  delay(1000); 
}




void loop() {


  
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    Serial.print("Received joint angles");

    // get comma indicies
    int firstComma = msg.indexOf(',');
    int secondComma = msg.indexOf(',', firstComma + 1);
    int thirdComma = msg.indexOf(',', secondComma + 1);

    // split string
    String field1 = msg.substring(0, firstComma);
    String field2 = msg.substring(firstComma + 1, secondComma);
    String field3 = msg.substring(secondComma + 1, thirdComma);
    String field4 = msg.substring(thirdComma + 1);

    // save angles as floats
    float a = field1.toFloat();
    float b = field2.toFloat();
    float c = field3.toFloat();
    float d = field4.toFloat();

    // output angles
    Serial.print("A: ");
    Serial.println(a);
    Serial.print("B: ");
    Serial.println(b);
    Serial.print("C: ");
    Serial.println(c);
    Serial.print("D: ");
    Serial.println(d);

    float a_steps = (a / 360.0) * motor_A_steps_rev;
    float b_steps = (b / 360.0) * motor_B_steps_rev;
    float c_steps = (c / 360.0) * motor_C_steps_rev;
    float d_steps = (d / 360.0) * motor_D_steps_rev;

    // Order of movement - A, D, C, B
    // Next - get movement required from homing positions

  }

}
