#include <Adafruit_NeoPixel.h>

//defining the pins
#define   MOTOR_A_IN1_PIN   10  // motor A pin 1 high for Forward
#define   MOTOR_B_IN1_PIN    6

int sensors[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // the array holds the analog pins connected to the sensors that detect the line

// PID Constants
float Kp = 0.5;  // Adjusts how strongly it corrects
float Kd = 0.2;  // Reduces wiggling
int baseSpeed = 250;  // Speed when moving straight

int lastError = 0;  // Stores previous error for derivative

void setup() {
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);  // sets the pins as output
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);

  analogWrite(MOTOR_A_IN1_PIN, 0); // the speed is set to 0 at the start
  analogWrite(MOTOR_B_IN1_PIN, 0);

  Serial.begin(9600);
}

void loop() { // did a loop so the code wont be hardcoded
  followLine(5, 2, 240, 240, 10);
}

void followLine(int leftSensor, int rightSensor, int leftWheelSpeed, int rightWheelSpeed, int time) {
  int error = analogRead(sensors[leftSensor]) - analogRead(sensors[rightSensor]);  
  // Difference between left-most and right-most sensor

  int derivative = error - lastError;  
  lastError = error;  // Calculates the rate of change of error 

  int correction = (Kp * error) + (Kd * derivative);  // Uses Proportional-Derivative control to determine how much the speed needs to change,  determines if it should move left or right 


  int leftSpeed = baseSpeed - correction;  // Adjusts motor speeds based on correction
  int rightSpeed = baseSpeed + correction;  

  leftSpeed = constrain(leftSpeed, 0, leftWheelSpeed);  //  this ensures that the motor speeds do not exceed the maximum allowed values.
  rightSpeed = constrain(rightSpeed, 0, rightWheelSpeed);  

  analogWrite(MOTOR_A_IN1_PIN, leftSpeed);
  analogWrite(MOTOR_B_IN3_PIN, rightSpeed);

  delay(time);  // Short delay for smoother response
}