//defining the pins
#define   MOTOR_A_IN1_PIN   10  // motor A pin 1 high for Forward
#define   MOTOR_A_IN2_PIN    9

#define   MOTOR_B_IN1_PIN    6
#define   MOTOR_B_IN2_PIN    5

#define   TRIGGER_PIN       12
#define   ECHO_PIN          13

#define   AVOID_OBJECT

// MOTOR A IS THE LEFT ONE, B IS THE RIGHT ONE
 
void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);

  pinMode(MOTOR_B_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN2_PIN, OUTPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Giving the motors 0 power first
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);

  analogWrite(MOTOR_B_IN1_PIN, 0);
  analogWrite(MOTOR_B_IN2_PIN, 0);

  delay(1000);
}

void loop() {
  //assigning the distance variable to the value returned by the getDistance() function
  long distance = getDistance();
  Serial.println(distance);
  
  moveForward(235, 255, 0, 0, 0);

  //if the distance between the ultrasonic sensor and the object is less than 25, avoid the object
#ifdef AVOID_OBJECT
  if (distance < 23) {
    Serial.print("Object detected ");
    Serial.print(getDistance());
    Serial.println(" cm away");

    moveLeft(0, 255, 0, 0, 400);
    delay(500);

    moveRight(255, 0, 0, 0, 500);
    delay(700);

    moveLeft(0, 255, 0, 0, 350);

    delay(200);
  }
#endif

  delay(500);
}

long getDistance() {
  //sending a short ultrasonic signal 
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH); 
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  //counting the duration of the signal and converting it to the distance in cm
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;  // Convert time to distance in cm
}

//The robot moves forward in a straight line
void moveForward(int a, int x, int b, int y, int time) {
  analogWrite(MOTOR_A_IN1_PIN, a);
  analogWrite(MOTOR_A_IN2_PIN, b);

  analogWrite(MOTOR_B_IN1_PIN, x);
  analogWrite(MOTOR_B_IN2_PIN, y);

  delay(time);
}

//The robot moves backwards in a straight line
void moveBackward(int a, int x, int b, int y) {
  analogWrite(MOTOR_A_IN1_PIN, a);
  analogWrite(MOTOR_A_IN2_PIN, b);

  analogWrite(MOTOR_B_IN1_PIN, x);
  analogWrite(MOTOR_B_IN2_PIN, y);
}

//The robot turns right by giving no power to the right wheel and giving maximum power to the left one
void moveRight(int a, int x, int b, int y, int time) {
  Serial.println("Moving Right...");

  analogWrite(MOTOR_A_IN1_PIN, a);
  analogWrite(MOTOR_A_IN2_PIN, b);

  analogWrite(MOTOR_B_IN1_PIN, x);
  analogWrite(MOTOR_B_IN2_PIN, y);

  delay(time);

  moveForward(235, 255, 0, 0, 400);
}

//The robot turns right by giving no power to the left wheel and giving maximum power to the right one
void moveLeft(int a, int x, int b, int y, int time) {
  Serial.println("Moving Left...");

  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);

  analogWrite(MOTOR_B_IN1_PIN, 255);
  analogWrite(MOTOR_B_IN2_PIN, 0);

  delay(time);

  moveForward(235, 255, 0, 0, 400);
}
