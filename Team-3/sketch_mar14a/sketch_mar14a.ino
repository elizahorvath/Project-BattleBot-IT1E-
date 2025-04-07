//################################
//#############bbnoEC#############
//#########Phisical Maze##########
//################################
#include <Adafruit_NeoPixel.h>

//Pins and other constant values:
const int GRIPPER                 = 10;//Gripper control pin
const int GRIPPER_OPEN            = 1800;//Amount of microsecounds for which GRIPPER pin should be high in order to open it
const int GRIPPER_CLOSE           = 1100;//Amount of microsecounds for which GRIPPER pin should be high in order to close it
const int MOTOR_A1                = 9; //Left motor backword
const int MOTOR_A2                = 6; //Left motor forword
const int MOTOR_B1                = 5; //Right motor backword
const int MOTOR_B2                = 3; //Right motor forword
const int FRONT_SONAR[2]          = {12, 11}; //Trigger and echo pins for front sonar
const int RIGHT_SONAR[2]          = {2, 13}; //Trigger and echo pins for front sonar
const int LEFT_SONAR[2]           = {A0, A1}; //Trigger and echo pins for front sonar
const int LINE_SENSOR[3]          = {A3, A4, A5}; //Three line sensors used to find the ending spot
const int NEOPIXEL_PIN            = A2; 
const int NUM_PIXELS              = 4;
const int TURN_90_RIGHT_IN_MILLIS = 800; //the exact amount of milliseconds needed for left wheel to be on at speed 10 to make 90deg turn right
const int TURN_90_LEFT_IN_MILLIS  = 720; //the exact amount of milliseconds needed for right wheel to be on at speed 10 to make 90deg turn left

bool _waitForStart;
bool _startSequence;
bool _maze;
bool _endSequence;
bool _isFirstSonarCheckDone;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel( 4, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800 );

// Returns distance from the closest object in cm
float sonar( int sonar[2] ){
  digitalWrite( sonar[0], LOW );
  delayMicroseconds( 2 );
  digitalWrite( sonar[0], HIGH );
  delayMicroseconds( 10 );
  digitalWrite( sonar[0], LOW );

  float duration = pulseIn( sonar[1], HIGH );
  return 0.034 * duration / 2;
}

// Functions goForwardR(), goForwardL(), goBackwardR() and goBackwardL() as an input take an inteeger between 0 to 20,
// where 0 makes motor stop and 20 sets it to max throttle

// Turns right motor forwards
void goForwardR( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite( MOTOR_B2, 0 );
  }
  else
  {
    analogWrite( MOTOR_B2, 0 );
    delay( 20 - pwm );
    analogWrite( MOTOR_B2, 255 );
    delay( pwm );
  }
      
}

// Turns left motor forward
void goForwardL( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite( MOTOR_A2, 0 );
  }
  else
  {
    analogWrite( MOTOR_A2, 0 );
    delay( 20 - pwm );
    analogWrite( MOTOR_A2, 255 );
    delay( pwm );
  }
}

// Turns left motor backward
void goBackwardL( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite( MOTOR_A1, 0 );
  }
  else
  {
    analogWrite( MOTOR_A1, 0 );
    delay( 20 - pwm );
    analogWrite( MOTOR_A1, 255 );
    delay( pwm );
  }
}

// Turns right motor backward
void goBackwardR( int pwm )
{
  if( pwm == 0 )
  {
    analogWrite( MOTOR_B1, 0 );
  }
  else
  {
    analogWrite( MOTOR_B1, 0 );
    delay( 20 - pwm );
    analogWrite( MOTOR_B1, 255 );
    delay( pwm );
  }
}

// Stops all the motors
void stopAll()
{
  analogWrite( MOTOR_A1, 0 );
  analogWrite( MOTOR_B1, 0 );
  analogWrite( MOTOR_A2, 0 );
  analogWrite( MOTOR_B2, 0 );
}

// The logic used to solve the maze. It outputs an int which corresponds to different actions based on data from ultrasonic sensors
int getDecision( float right, float left, float front )
{
  if( right > 18 ) //turn right
  {
    return 1;
  }

  if( right <= 18 )
  {
    if( front <= 15 && left > 15 ) //turn left
    {
      return 3;
    }

    if( front > 15 )
    {
      if( right < 7 ) //correct to left
      {
        return 5;
      }

      if( right > 9 ) //correct to right
      {
        return 6;
      }

      else
      {
        return 2; //forward
      }
    }

    if( front <= 8 && left <= 15 ) //go back
    {
      return 4;
    }
  } 
}

// Sets the position of gripper servo
void gripper( int state )
{
  static unsigned long servoTimer;
  static int lastState;
  if( micros() > servoTimer < 20000 )
  {
    if(state > 0)
    {
      lastState = state;
    }

    else
    {
      state = lastState;
    }

    digitalWrite( GRIPPER, HIGH );
    delayMicroseconds( state );
    digitalWrite( GRIPPER, LOW );
    servoTimer = millis() + 20;
  }
}

// Sets the colors of Neopixels
void setAllLights( int r, int g, int b )
{
  for( int i = 0; i <= NUM_PIXELS; i++ )
  {
    pixels.setPixelColor( i, pixels.Color( r, g, b ) );
  }

  pixels.show();
}

// Calculates the edge value of line sensors between white and black surface
int getEdgeValue( int sensorMin, int sensorMax )
{
  return sensorMin + sensorMax / 2;
}

void setup() {
  // Set motors to output
  pinMode( MOTOR_A1, OUTPUT );
  pinMode( MOTOR_B1, OUTPUT );
  pinMode( MOTOR_A2, OUTPUT );
  pinMode( MOTOR_B2, OUTPUT );
  pinMode( RIGHT_SONAR[0], OUTPUT );
  pinMode( FRONT_SONAR[0], OUTPUT );
  pinMode( LEFT_SONAR[0], OUTPUT );
  pinMode( RIGHT_SONAR[1], INPUT );
  pinMode( FRONT_SONAR[1], INPUT );
  pinMode( LEFT_SONAR[1], INPUT );

  // Set motor outputs to 0;
  analogWrite( MOTOR_A1, 0 );
  analogWrite( MOTOR_B1, 0 );
  analogWrite( MOTOR_A2, 0 );
  analogWrite( MOTOR_B2, 0 );

  Serial.begin( 9600 );
  pixels.begin();

  _startSequence         = 0;
  _maze                  = 0;
  _endSequence           = 0;
  _waitForStart          = 1;
  _isFirstSonarCheckDone = false;

  setAllLights( 255, 0, 0 );
}

void loop() {
  float rightSonar; //Distance measured by right ultrasonic sensor
  float leftSonar; //Distance measured by left ultrasonic sensor
  float frontSonar; //Distance measured by front ultrasonic sensor
  long timer; // Timer used for timing different movements in the maze using millis() function
  int edgeValue; //Stores the edge value after calibration
  
  // Standby
  if( _waitForStart == 1 )
  {
    bool isObjectReal = false; //Used to validate if signal received by front sonar is a real object in front of the robot or noise from another robot
    int threshold     = 30; //The minimal distance from the robot that the object must be in in order to activate the starting sequence
    
    // frontSonar gets updated once every second
    gripper( GRIPPER_OPEN );
    if( millis() > timer )
    {
      _isFirstSonarCheckDone = true;
      frontSonar             = sonar( FRONT_SONAR );
      timer                  = millis() + 1000;
    }

    // In case that frontSonar doesn't update in time when arduino is turned on the value is set to 100
    if( _isFirstSonarCheckDone == false )
    {
      frontSonar = 100;
    }

    // If robot detects an object in front of itself, it runns three checks in different 
    // intervals to check if it is a real object or noise from another robot
    if( frontSonar < threshold && isObjectReal == false )
    {
      delay( 50 );
      if( sonar( FRONT_SONAR ) < threshold )
      {
        delay( 30 );
        if( sonar( FRONT_SONAR ) < threshold )
        {
          isObjectReal = true;
        }
      }
    }

    // If object is real the robot starts the starting sequence
    if( isObjectReal )
    {
      _waitForStart  = 0;
      _startSequence = 1;
    }
  }
  
  // Starting sequence
  if( _startSequence == 1 )
  {
    setAllLights( 0, 0, 255 );
    delay( 1000 ); // Robot waits 1 second for the previous robot to back up before starting
    int sensorMin; // Stores the minimal value read by line sensor during calibration
    int sensorMax; // Stores the maximal value read by line sensor during calibration
    
    // Robot goes forward towards the object and at the same time it collects the maximal and 
    // minimal values of line sensor by passing by three black strips on the floor
    timer = millis();
    while( millis() - timer < 1500 )
    {
      gripper( GRIPPER_OPEN );
      goForwardR( 10 );
      goForwardL( 12 );
      goBackwardR( 0 );
      goBackwardL( 0 );
      if( analogRead( LINE_SENSOR[1] ) < sensorMin )
      {
        sensorMin = analogRead( LINE_SENSOR[1] );
      }

      if( analogRead( LINE_SENSOR[1] ) > sensorMax )
      {
        sensorMax = analogRead( LINE_SENSOR[1] );
      }
    }

    // The edge value is calculated based on data colected during calibration
    edgeValue = getEdgeValue( sensorMin, sensorMax );

    // Ropot closes the gripper on the object and turns 90 degrees left
    timer = millis();
    while( millis() - timer < TURN_90_LEFT_IN_MILLIS )
    {
      gripper( GRIPPER_CLOSE );
      goForwardR( 20 );
      goForwardL( 0 );
      goBackwardR( 0 );
      goBackwardL( 0 );

      setAllLights( 0, 255, 0 );
    }

    // Robot goes forwards to the entrance of the maze
    timer = millis();
    while( millis() - timer < 1300 )
    {
      gripper( GRIPPER_CLOSE );
      goForwardR( 10 );
      goForwardL( 10 );
      goBackwardR( 0 );
      goBackwardL( 0 );
    }

    _startSequence = 0;
    _maze          = 1;
  }
  
  // If robot detects the black sqere on the flor it starts the end sequence
  if( _maze == 1 && ( analogRead( LINE_SENSOR[0] ) > edgeValue || analogRead( LINE_SENSOR[1] ) > edgeValue || analogRead( LINE_SENSOR[2] ) > edgeValue ) )
  {
    timer = millis();
    while( millis() - timer < 200 )
    {
      goForwardR( 10 );
      goForwardL( 10 );
      goBackwardR( 0 );
      goBackwardL( 0 );
    }

    if( analogRead( LINE_SENSOR[0] ) > edgeValue || analogRead( LINE_SENSOR[1] ) > edgeValue || analogRead( LINE_SENSOR[2] ) > edgeValue )
    {
      _maze        = 0;
      _endSequence = 1;
    }
  }

  // Solving the maze with right hand rule
  if( _maze == 1 )
  {
    gripper( GRIPPER_CLOSE );

    // Robot stores distances messured by every ultrasonic sensor 
    rightSonar = sonar( RIGHT_SONAR );
    leftSonar = sonar( LEFT_SONAR );
    frontSonar = sonar( FRONT_SONAR );

    // Based on the positions of walls around the robot it decides on what move it should do
    switch( getDecision( rightSonar, leftSonar, frontSonar ) )
    {
      case 1: // Turn right
      setAllLights( 255, 255, 0 );
      timer = millis();
      while( millis() - timer < 400 )
      {
        goForwardR( 10 );
        goForwardL( 10 );
        goBackwardR( 0 );
        goBackwardL( 0 );
      }

      gripper( GRIPPER_CLOSE );
      timer = millis();
      while( millis() - timer < TURN_90_RIGHT_IN_MILLIS )
      {
        goForwardR( 0 );
        goForwardL( 20 );
        goBackwardR( 0 );
        goBackwardL( 0 );
      }

      gripper( GRIPPER_CLOSE );
      timer = millis();
      while( millis() - timer < 300 )
      {
        goForwardR( 10 );
        goForwardL( 10 );
        goBackwardR( 0 );
        goBackwardL( 0 );
      }

      break;

      case 2: // Go forward
      setAllLights( 255, 165, 0 );
      goForwardR( 10 );
      goForwardL( 10 );
      goBackwardR( 0 );
      goBackwardL( 0 );
      break;

      case 3: // Turn left
      setAllLights( 255, 0, 255 );
      timer = millis();
      while( millis() - timer < TURN_90_LEFT_IN_MILLIS )
      {
        goForwardR( 20 );
        goForwardL( 0 );
        goBackwardR( 0 );
        goBackwardL( 0 );
      }

      gripper( GRIPPER_CLOSE );
      timer = millis();
      while( millis() - timer < 600 )
      {
        goForwardR( 10 );
        goForwardL( 10 );
        goBackwardR( 0 );
        goBackwardL( 0 );
      }

      break;

      case 4: // Go back
      setAllLights( 165, 0, 0 );
      timer = millis();
      while( millis() - timer < 800 )
      {
        goForwardR( 0 );
        goForwardL( 0 );
        goBackwardR( 0 );
        goBackwardL( 20 );
      }

      gripper( GRIPPER_CLOSE );
      timer = millis();
      while( millis() - timer < 900 )
      {
        goForwardR( 20 );
        goForwardL( 0 );
        goBackwardR( 0 );
        goBackwardL( 0 );
      }

      gripper( GRIPPER_CLOSE );
      timer = millis();
      while( millis() - timer < 700 )
      {
        goForwardR( 0 );
        goForwardL( 0 );
        goBackwardR( 10 );
        goBackwardL( 10 );
      }
      
      break;

      case 5: // Correct to left
      goForwardR( 20 );
      goForwardL( 10 );
      goBackwardR( 0 );
      goBackwardL( 0 );
      break;

      case 6: // Correct to right
      goForwardR( 10 );
      goForwardL( 20 );
      goBackwardR( 0 );
      goBackwardL( 0 );
      break;
    }
  }

  // End sequence
  if( _endSequence == 1 )
  {
    stopAll();
    gripper( GRIPPER_OPEN );

    // After droping the object robot backs
    timer = millis();
    while( millis() - timer < 700 )
    {
      goForwardR( 0 );
      goForwardL( 0 );
      goBackwardR( 10 );
      goBackwardL( 10 );
    }

    // After that the robot stops everything
    stopAll();
    _endSequence = 0;
  }
}