//MOTOR CONTROL PROGRAM FOR ME 670 FINAL PROJECT
//This program allows a user to input a motor power level with the serial monitor, and have the motor spin to that power
//This version allows for proportional control
//Simon Popecki
//James Skinner
//Jesse Feng

//Developed on Arduino UNO
//For: Thunderbird 18 ESC; MEGA Motor ACn 16/15/8

#include <Servo.h>

const uint8_t SIGNAL = 3; //this is the pin on the Arduino producing the signal for the ESC (does not need to be a PWM output)
const uint32_t BAUD = 9600; //rate of communication with the arduino

Servo THUNDERBIRD; //Creating a servo object for the thunderbird 18 ESC (it is controlled the same way a servo would be)

void setup()
{
  THUNDERBIRD.attach(SIGNAL); //this binds the servo object to a specific pin, the servo object is used instead of a pin number from here on out
  Serial.begin(BAUD); //start a serial communication - make sure that the serial monitor is set to a BAUD of 9600, and no line endings!
  Serial.println("Enter a motor speed (percentage of full throttle):");
  THUNDERBIRD.writeMicroseconds(1000); //initializing the ESC - it won't take any commands until it sees zero throttle (1000 ms). This is done in setup so the user doesn't have to do it in the terminal.
}

void loop()
{
  while (Serial.available())
  {
    uint16_t target = Serial.parseInt(); //whatever number the user just typed in is the new motor pulse speed IN PERCENT!
    Serial.print("New Speed: ");
    Serial.print(target);
    Serial.print("%");
    Serial.print('\n'); //new line for the next output
    target = (target*10)+1000; //converting percentage to a pulse width for use with theThunderbird 18 ESC  
    THUNDERBIRD.writeMicroseconds(pulse); //Sending the command to the ESC - note that this takes about a second for an Arduino Uno to do, a more powerful microcontroller is recommended
  }

}
