const int MOTOR_A1       = 9; //Left motor backword
const int MOTOR_A2       = 6; //Left motor forword
const int MOTOR_B1       = 5; //Right motor backword
const int MOTOR_B2       = 3; //Right motor forword
const int SENSOR_A       = 4; //Left rotation sensor
const int SENSOR_B       = 7; //Right rotation sensor
const int FRONT_SONAR[2] = {12, 11}; //Trigger and echo pins for front sonar
const int RIGHT_SONAR[2] = {2, 13}; //Trigger and echo pins for front sonar
const int LEFT_SONAR[2]  = {A0, A1}; //Trigger and echo pins for front sonar

//Amount of signals received from rotation sensor. Every signal indicates 1/20 of full rotation of the wheel
int _countFractionR = 0;
int _countFractionL = 0;

//States of the sensors
bool _stateR = 0;
bool _stateL = 0;

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
    if(front <= 20 && left > 15)//turn left
    {
      return 3;
    }

    if(front > 10)
    {
      if(right < 6)//correct to left
      {
        return 5;
      }

      if(right > 9)//correct to right
      {
        return 6;
      }

      else
      {
        return 2; //forward
      }
    }

    

    if(front <= 20 && left <= 15)//go back
    {
      return 4;
    }
  }
  
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
}

void loop() {
  float rightSonar;
  float leftSonar;
  float frontSonar;
  float timer;
  int turn90 = 17;

  // unsigned static long sonarTimer;
  // if(millis() - sonarTimer > 800)
  // {
    rightSonar = sonar(RIGHT_SONAR);
    leftSonar = sonar(LEFT_SONAR);
    frontSonar = sonar(FRONT_SONAR);
    //sonarTimer = millis();
  //}

  Serial.print("Right:");
  Serial.println(rightSonar);
  Serial.print("Left:");
  Serial.println(leftSonar);
  Serial.print("Front:");
  Serial.println(frontSonar);

  switch(getDecision(rightSonar, leftSonar, frontSonar))
  {
    case 1: //turn right
    timer = millis();
    while(millis() - timer < 300)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }

    _countFractionR = 0;
    _countFractionL = 0;
    while(_countFractionL < turn90)
    {
      goForwardR(0);
      goForwardL(20);
      rotationsR();
      rotationsL();
      goBackwardR(0);
      goBackwardL(0);
    }

    timer = millis();
      while(millis() - timer < 700)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }

    break;

    case 2: //forward
    timer = millis();
      while(millis() - timer < 100)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }
      
    break;

    case 3: //turn left
    _countFractionR = 0;
    _countFractionL = 0;
    while(_countFractionR < turn90)
    {
      goForwardR(20);
      rotationsR();
      rotationsL();
      goForwardL(0);
      goBackwardR(0);
      goBackwardL(0);
    }

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
    timer = millis();
    while(millis() - timer < 1700)
    {
      goForwardR(0);
      goForwardL(0);
      goBackwardR(10);
      goBackwardL(10);
    }

    _countFractionR = 0;
    _countFractionL = 0;
    while(_countFractionL < turn90)
    {
      goForwardR(0);
      rotationsR();
      rotationsL();
      goForwardL(0);
      goBackwardR(0);
      goBackwardL(20);
    }

    
    frontSonar = sonar(FRONT_SONAR);

    if(frontSonar > 8)
    {
      timer = millis();
      while(millis() - timer < 600)
      {
        goForwardR(10);
        goForwardL(10);
        goBackwardR(0);
        goBackwardL(0);
      }
    }

    else
    {
      _countFractionR = 0;
      _countFractionL = 0;
      while(_countFractionR < 9 && _countFractionL < 9)
      {
        rotationsR();
        rotationsL();
        goForwardR(10);
        rotationsR();
        rotationsL();
        goForwardL(0);
        goBackwardR(0);
        goBackwardL(10);
      }
    }
    
    break;

    case 5: //correct to left
    goForwardR(20);
    goForwardL(5);
    goBackwardR(0);
    goBackwardL(0);
    break;

    case 6: //correct to right
    goForwardR(5);
    goForwardL(20);
    goBackwardR(0);
    goBackwardL(0);
    break;
  }
}
