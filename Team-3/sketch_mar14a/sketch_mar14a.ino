//################################
//#############bbnoEC#############
//##########Maze runner###########
//################################
#include <Adafruit_NeoPixel.h>

const int GRIPPER       = 10;//Gripper control pin
const int GRIPPER_OPEN  = 1800;//Amount of microsecounds for which GRIPPER pin should be high in order to open it
const int GRIPPER_CLOSE = 1100;//Amount of microsecounds for which GRIPPER pin should be high in order to close it
const int MOTOR_A1       = 9; //Left motor backword
const int MOTOR_A2       = 6; //Left motor forword
const int MOTOR_B1       = 5; //Right motor backword
const int MOTOR_B2       = 3; //Right motor forword
const int SENSOR_A       = 4; //Left rotation sensor
const int SENSOR_B       = 7; //Right rotation sensor
const int FRONT_SONAR[2] = {12, 11}; //Trigger and echo pins for front sonar
const int RIGHT_SONAR[2] = {2, 13}; //Trigger and echo pins for front sonar
const int LEFT_SONAR[2]  = {A0, A1}; //Trigger and echo pins for front sonar
const int NEOPIXEL_PIN    = A2; 
const int NUM_PIXELS       = 4;

//Amount of signals received from rotation sensor. Every signal indicates 1/20 of full rotation of the wheel
int _countFractionR = 0;
int _countFractionL = 0;

//States of the sensors
bool _stateR = 0;
bool _stateL = 0;

unsigned long previousMillis = 0;

bool _waitForStart;
bool _startSequence;
bool _maze;
bool _endSequence;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(4, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);

//returns distance from the closest object in cm
float sonar(int sonar[2]){
  digitalWrite(sonar[0], LOW);
  delayMicroseconds(2);
  digitalWrite(sonar[0], HIGH);
  delayMicroseconds(10);
  digitalWrite(sonar[0], LOW);

  float duration = pulseIn(sonar[1], HIGH);
  return 0.034 * duration / 2;
}

//Turns right motor goes forwards
void goForwardR( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite(MOTOR_B2, 0);
  }
  else
  {
    analogWrite(MOTOR_B2, 0);
    delay(20-pwm);
    analogWrite(MOTOR_B2, 255);
    delay(pwm);
  }
      
}

//turns left motor forward
void goForwardL( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite(MOTOR_A2, 0);
  }
  else
  {
    analogWrite(MOTOR_A2, 0);
    delay(20-pwm);
    analogWrite(MOTOR_A2, 255);
    delay(pwm);
  }
}

//turns left motor backward
void goBackwardL( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite(MOTOR_A1, 0);
  }
  else
  {
    analogWrite(MOTOR_A1, 0);
    delay(20-pwm);
    analogWrite(MOTOR_A1, 255);
    delay(pwm);
  }
}

//turns right motor backward
void goBackwardR( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite(MOTOR_B1, 0);
  }
  else
  {
    analogWrite(MOTOR_B1, 0);
    delay(20-pwm);
    analogWrite(MOTOR_B1, 255);
    delay(pwm);
  }
}

//Stops all the motors
void stopAll()
{
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);
}

//counts the amount of rotations on right motor
void rotationsR()
{
  if( digitalRead(SENSOR_A) == HIGH && _stateR == 0 )
  {
    _stateR = 1;
    _countFractionR++;
  }
  if( digitalRead(SENSOR_A) == LOW && _stateR == 1 )
  {
    _stateR = 0;
  }
}

//counts the amount of rotations on left motor
void rotationsL()
{
    if( digitalRead(SENSOR_B) == HIGH && _stateL == 0 )
    {
      _stateL = 1;
      _countFractionL++;
    }
    if( digitalRead(SENSOR_B) == LOW && _stateL == 1 )
    {
      _stateL = 0;
    }
    
}

int getDecision(float right, float left, float front)
{
  if(right > 18) //turn right
  {
    return 1;
  }

  if(right <= 18)
  {
    if(front <= 15 && left > 15)//turn left
    {
      return 3;
    }

    if(front > 15)
    {
      if(right < 5)//correct to left
      {
        return 5;
      }

      if(right > 7)//correct to right
      {
        return 6;
      }

      else
      {
        return 2; //forward
      }
    }

    if(front <= 8 && left <= 15)//go back
    {
      return 4;
    }
  } 
}

//Sets the position of gripper servo
void gripper(int state)
{
  static unsigned long servoTimer;
  static int lastState;
  if(micros() > servoTimer < 20000)
  {
    if(state > 0)
    {
      lastState = state;
    }

    else
    {
      state = lastState;
    }

    digitalWrite(GRIPPER, HIGH);
    delayMicroseconds(state);
    digitalWrite(GRIPPER, LOW);
    servoTimer = millis() + 20;
  }
}

void setAllLights(int r, int g, int b)
{
  for(int i = 0; i <= NUM_PIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }

  pixels.show();
}

void setup() {
  //Set motors to output
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(RIGHT_SONAR[0], OUTPUT);
  pinMode(FRONT_SONAR[0], OUTPUT);
  pinMode(LEFT_SONAR[0], OUTPUT);
  pinMode(RIGHT_SONAR[1], INPUT);
  pinMode(FRONT_SONAR[1], INPUT);
  pinMode(LEFT_SONAR[1], INPUT);

  //Set rotation sensors to input
  pinMode(SENSOR_A, INPUT);
  pinMode(SENSOR_B, INPUT);

  //Set motor outputs to 0;
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B2, 0);

  Serial.begin(9600);

  pixels.begin();

  _startSequence = 0;
  _maze = 0;
  _endSequence = 0;
  _waitForStart = 1;

  if( digitalRead(SENSOR_A) == HIGH )
  {
    _stateL = 1;
  }
  else
  {
    _stateL = 0;
  }

  if( digitalRead(SENSOR_B) == HIGH )
  {
    _stateR = 1;
  }
  else
  {
    _stateR = 0;
  }

  setAllLights(255, 0, 0);
}

void loop() {
  float rightSonar;
  float leftSonar;
  float frontSonar;
  long timer;
  int turn90millisRight = 800;
  int turn90millisLeft = 720;
  
  if(_waitForStart == 1)
  {
    gripper(GRIPPER_OPEN);
    frontSonar = 10000000000;
    if(millis() - timer > 1000)
    {
      frontSonar = sonar(FRONT_SONAR);
      timer = millis();
    }

    if(frontSonar < 38)
    {
      _waitForStart = 0;
      _startSequence = 1;
    }
  }
  
  if(_startSequence == 1)
  {
    setAllLights(0, 0, 255);
    delay(1000);
    
    timer = millis();
    while(millis() - timer < 1580)
    {
      gripper(GRIPPER_OPEN);
      goForwardR(10);
      goForwardL(12);
      goBackwardR(0);
      goBackwardL(0);
    }

    timer = millis();
    while(millis() - timer < turn90millisLeft)
    {
      gripper(GRIPPER_CLOSE);
      goForwardR(20);
      goForwardL(0);
      goBackwardR(0);
      goBackwardL(0);

      setAllLights(0, 255, 0);
    }

    timer = millis();
    while(millis() - timer < 1300)
    {
      gripper(GRIPPER_CLOSE);
      goForwardR(10);
      goForwardL(10);
      goBackwardR(0);
      goBackwardL(0);
    }

    _startSequence = 0;
    _maze = 1;
  }

  if(_maze == 1)
  {
    gripper(GRIPPER_CLOSE);
    rightSonar = sonar(RIGHT_SONAR);
    leftSonar = sonar(LEFT_SONAR);
    frontSonar = sonar(FRONT_SONAR);
      
    // Serial.print("Right:");
    // Serial.println(rightSonar);
    // Serial.print("Left:");
    // Serial.println(leftSonar);
    // Serial.print("Front:");
    // Serial.println(frontSonar);

    switch(getDecision(rightSonar, leftSonar, frontSonar))
    {
      case 1: //turn right
      setAllLights(255, 255, 0);
      timer = millis();
      while(millis() - timer < 400)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }

      gripper(GRIPPER_CLOSE);
      timer = millis();
      while(millis() - timer < turn90millisRight)
      {
        goForwardR(0);
        goForwardL(20);
        goBackwardR(0);
        goBackwardL(0);
      }

      gripper(GRIPPER_CLOSE);
      timer = millis();
      while(millis() - timer < 300)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }

      break;

      case 2: //forward
      setAllLights(255, 165, 0);
      goForwardR(10);
      goForwardL(10);
      goBackwardR(0);
      goBackwardL(0);
      break;

      case 3: //turn left
      setAllLights(255, 0, 255);
      timer = millis();
      while(millis() - timer < turn90millisLeft)
      {
        goForwardR(20);
        goForwardL(0);
        goBackwardR(0);
        goBackwardL(0);
      }

      gripper(GRIPPER_CLOSE);
      timer = millis();
      while(millis() - timer < 600)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }

      break;

      case 4: //go back
      setAllLights(165, 0, 0);
      timer = millis();
      while(millis() - timer < 800)
      {
        goForwardR(0);
        goForwardL(0);
        goBackwardR(0);
        goBackwardL(20);
      }

      gripper(GRIPPER_CLOSE);
      timer = millis();
      while(millis() - timer < 900)
      {
        goForwardR(20);
        goForwardL(0);
        goBackwardR(0);
        goBackwardL(0);
      }

      gripper(GRIPPER_CLOSE);
      timer = millis();
      while(millis() - timer < 700)
      {
        goForwardR(0);
        goForwardL(0);
        goBackwardR(10);
        goBackwardL(10);
      }
      
      break;

      case 5: //correct to left
      goForwardR(20);
      goForwardL(10);
      goBackwardR(0);
      goBackwardL(0);
      break;

      case 6: //correct to right
      goForwardR(10);
      goForwardL(20);
      goBackwardR(0);
      goBackwardL(0);
      break;
    }
  }
}
