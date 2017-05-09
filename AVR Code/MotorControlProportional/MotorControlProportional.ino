//MOTOR CONTROL PROGRAM FOR ME 670 FINAL PROJECT
//This program allows a user to input a motor power level with the serial monitor, and have the motor spin to that power
//This version allows for proportional control
//Simon Popecki
//James Skinner
//Jesse Feng

//Developed on Arduino UNO
//For: Thunderbird 18 ESC; MEGA Motor ACn 16/15/8
//This program is for tempermental electronic speed controllers that require the 50 Hz PWM signal given by hobby receivers (1-2 ms on, 18-19 ms off)
//A pulse width of 2 ms on represents full throttle, 1 ms 

#include <Servo.h>
#include <stdint.h>

const uint8_t SIGNAL = 3; //this is the pin on the Arduino producing the signal for the ESC (does not need to be a PWM output pin)
const uint32_t BAUD = 9600; //rate of communication with the arduino
const uint8_t RESPONSETUNING = 4; //this is a proportional control parameter, it adjusts how fast the motor will reach the desired speed. Higher values decrease the response speed
const uint16_t TIMETUNING = 200; //this is another proportional control parameter, it adjusts how long the motor spins at a certain value during speed transient
uint16_t current = 1000; //the current pulse value - zero throttle to arm the ESC - this value must be 1000 microseconds during start up, but is changed later as the program runs when the variable is used for proportional control

Servo THUNDERBIRD; //Creating a servo object for the thunderbird 18 ESC (it is controlled the same way a servo would be)

void setup()
{
  THUNDERBIRD.attach(SIGNAL); //this binds the servo object to a specific pin, the servo object is used instead of a pin number from here on out
  Serial.begin(BAUD); //start a serial communication - make sure that the serial monitor is set to a BAUD of 9600, and no line endings!
  Serial.println("Enter a motor speed (percentage of full throttle):");
  THUNDERBIRD.writeMicroseconds(current); //initializing the ESC - it won't take any commands until it sees zero throttle (1000 microseconds). This is done in setup so the user doesn't have to do it in the terminal.
}

void loop()
{
  while (Serial.available())
  {
    uint16_t target = Serial.parseInt(); //whatever number the user just typed in is the new motor pulse speed IN PERCENT!
    Serial.print("New Speed: ");
    Serial.print(target); //displaying the percentage value to the user before it is converted to a pulse width for the ESC to understand
    Serial.print("%");
    Serial.print('\n'); //new line for the next output
    target = (target*10)+1000; //converting percentage to a pulse width for use with theThunderbird 18 ESC

    int16_t difference = target-current; //finding the difference between the target speed (pulse length) and the current speed (pulse length) - the units are pulse width difference in microseconds
    
    while (difference != 0)
    {
      int16_t response = difference/RESPONSETUNING; //response can be positive or negative, and uses a tuning factor to generate a value to be added or subtracted from the current pulse width to get closer to the target value
      current = current+response; //updating the new speed of the motor by factoring in the response modifier
      THUNDERBIRD.writeMicroseconds(current);
      difference = target-current;
      //Serial.println(difference); //the difference between the current speed setting and target speed setting can be displayed if desired
      delay(TIMETUNING); 
      if ((difference <= 5) && (difference >= -5)) //if the difference is small enough then just ignore it and go straight to the target value
        {
          THUNDERBIRD.writeMicroseconds(target); //go straight to the target pulse width before terminating the while loop
          current = target;
          difference = 0; //terminates the while loop and resets the difference variable for further use
        }
    }
  }
}
