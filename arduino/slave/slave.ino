#include <Wire.h>
#include <Servo.h>

// create instances
Servo pump;
Servo valve;

// pump: 0 = OFF, 180 = ON
// valve: 0 = CLOSED, 180 = OPEN

volatile int command = 0;  // shared between ISR and loop

void setup() { 
  // I2C slave config
  Wire.begin(8);
  Wire.onReceive(receiveEvent);

  // pump is on pin 9, valve is on pin 8  
  pump.attach(9);
  valve.attach(8);

  pump.write(0);  
  valve.write(0);
} 

void loop() { 
  static int lastCommand = 0;

  if (command != lastCommand) {
    lastCommand = command;

    if (command == 1) {
      pump.write(0);
      valve.write(0);
    } 
    else if (command == 2) {
      pump.write(180);
      valve.write(0);
    } 
    else if (command == 3) {
      pump.write(0);
      valve.write(180);
    } 
    else if (command == 4) {
      pump.write(180);
      valve.write(180);
    }
  }
}

void receiveEvent(int noEvents) {
  if (Wire.available()) {
    command = Wire.read(); 
  }
}
