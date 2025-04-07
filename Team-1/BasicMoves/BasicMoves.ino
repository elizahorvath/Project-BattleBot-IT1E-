const int MOTOR_A_IN1_PIN = 10;
const int MOTOR_A_IN2_PIN = 9;
const int MOTOR_B_IN3_PIN = 6;
const int MOTOR_B_IN4_PIN = 5;


void setup() {
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN4_PIN, OUTPUT);
  pinMode(MOTOR_B_IN3_PIN, OUTPUT);
  analogWrite(MOTOR_A_IN1_PIN, 0);
}

void loop() {
  moveForward();
  delay(1500);
  moveBackward();
  delay(1500);
  moveForward();
  delay(1500);
  turnRight();
  moveForward();
  delay(1500);
  turnLeft();
  moveForward();
  delay(1500);
}

void moveForward() {
  analogWrite(MOTOR_A_IN1_PIN, 240);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN3_PIN, 255);
  analogWrite(MOTOR_B_IN4_PIN, 0);

}
  
void turnLeft() {
  analogWrite(MOTOR_A_IN1_PIN, 255);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN3_PIN, 0);
  analogWrite(MOTOR_B_IN4_PIN, 0);

  delay(900);
}

void turnRight() {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN3_PIN, 255);
  analogWrite(MOTOR_B_IN4_PIN, 0);

  delay(900);
}

void moveBackward() {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 240);
  analogWrite(MOTOR_B_IN3_PIN, 0);
  analogWrite(MOTOR_B_IN4_PIN, 255);

}
