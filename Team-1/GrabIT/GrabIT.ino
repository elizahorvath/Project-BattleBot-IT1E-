#define SERVO_PIN         4
#define MOTOR_A_IN1_PIN   10  // LEFT motor - FORWARD
#define MOTOR_B_IN1_PIN   6   // RIGHT motor - FORWARD

#define GRIPPER_OPEN      1800   // Pulse length for open gripper
#define GRIPPER_CLOSE     1100  // Pulse length for closed gripper

unsigned long prevTime = 0;  // Timer for state transitions
int state = 0;  // Tracks which step the robot is in

void setup() {
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);

  analogWrite(MOTOR_A_IN1_PIN, 0);  // Giving the motors 0 power initially
  analogWrite(MOTOR_B_IN1_PIN, 0);
  digitalWrite(SERVO_PIN, LOW);

  prevTime = millis();  // Start the timer
}

void loop() {
  unsigned long currentTime = millis();

  switch (state) {
    case 0:  // Opening the gripper when the robot is first turned on
      gripper(GRIPPER_OPEN);
      prevTime = currentTime;
      state = 1;
      break;
    
    case 1:  // Waiting 1 second before closing the gripper
      if (currentTime - prevTime >= 1000) {
          state = 2;
      }
      break;

    case 2:  // Closing the gripper
      gripper(GRIPPER_CLOSE);
      prevTime = currentTime;
      state = 3;
      break;

    case 3:  // Waiting 1 more second before opening the gripper again
      if (currentTime - prevTime >= 1000) {
          state = 4;
      }
      break;

    case 4:  // Opening the gripper again
      gripper(GRIPPER_OPEN);
      prevTime = currentTime;
      state = 5;
      break;

    case 5:  // Driving forward
      driveForward(250, 255);
      if (currentTime - prevTime >= 2000) {
          state = 6;
      }
      break;

    case 6:  // Closing the gripper while driving
      gripper(GRIPPER_CLOSE);
      driveForward(230, 255);  // Keep moving
      break;
  }

    // Continuously update the gripper every 20ms to keep servo active
    gripper(-1);
}

void gripper(int pulse) {
  static unsigned long timer;
  static int lastPulse = GRIPPER_OPEN;  // Default position

  if (pulse > 0) {
    lastPulse = pulse;  // Store the last valid pulse value
  }

  if (millis() - timer >= 20) {  // Refresh servo every 20ms
    timer = millis();
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(lastPulse);
    digitalWrite(SERVO_PIN, LOW);
  }
}

void driveForward(int a, int b) {
  analogWrite(MOTOR_A_IN1_PIN, a);
  analogWrite(MOTOR_B_IN1_PIN, b);
}
