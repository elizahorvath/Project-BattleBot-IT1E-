// Define motor and servo pins
#define     SERVO_PIN            4
#define     MOTOR_A_IN1_PIN     10    // LEFT motor - FORWARD
#define     MOTOR_B_IN1_PIN      6    // RIGHT motor - FORWARD

#define     MOTOR_A_IN2_PIN      3    // LEFT motor - BACKWARDS
#define     MOTOR_B_IN2_PIN      5    // RIGHT motor - BACKWARDS

#define     TRIGGER_PIN         12    // TRIGGER pin  for the ULTRASONIC sensor - sends signals
#define     ECHO_PIN            13    // ECHO pin for the ULTRASONIC sensor - receives the signals

#define     GRIPPER_OPEN      1800    // Pulse length for open gripper
#define     GRIPPER_CLOSE     1100    // Pulse length for closed gripper

// Sensor array for line following
int sensors[] = {A0, A1, A2, A3, A4, A5, A6, A7};
float Kp = 0.6; 
float Kd = 0.5; // PD control parameters
int baseSpeed = 240; 
int lastError = 0; // Base speed and last error value for PD controller

unsigned long timer = 0; // Timer variable
int state = 0; // State variable to track different steps
bool gripperClosed = false; // Flag for gripper state
bool blackCrossedOnce = false; // Flag to track first black area crossing
int blackCrossCount = 0; // Counter for black area crossings
unsigned long blackStartTime = 0; // Timer for black area detection
bool movingBackward = false; // Flag to track backward movement
unsigned long backwardStartTime = 0; // Timer for moving backwards after dropping the cone
bool stopRobot = false; // Flag to stop the robot completely once the course is completed

void setup() {
  pinMode(SERVO_PIN, OUTPUT);

  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Giving the motors 0 power first
  stopMotors();
  digitalWrite(SERVO_PIN, LOW);
}

void loop() {
  if (stopRobot) {
    return; // Ensures robot stops at the end
  }

  long distance = getDistance();
  unsigned long now = millis();

  if (distance > 20) {
    if (state == 0) {
      timer = now;
      state = 1;
    }

    // Move forward for 1 second
    if (state == 1 && now - timer < 1000) {
      moveForward(240, 255);
    } else if (state == 1) {
      stopMotors();
      timer = now;
      state = 2;
    }
    
    // Close the gripper after stopping
    if (state == 2 && now - timer < 500) {
      gripper(GRIPPER_CLOSE);
    } else if (state == 2) {
      timer = now;
      state = 3;
    }

    // Turn left for 900ms
    if (state == 3 && now - timer < 800) {
      moveForward(0, 255);
    } else if (state == 3) {
      stopMotors();
      timer = now;
      state = 4;
    }
    
    // Small delay before starting line-following
    if (state == 4 && now - timer < 100) {
      moveForward(240, 255);
    } else if (state == 4) {
      timer = now;
      state = 5;
    }

    if (state >= 5) {
      if (allSensorsBlack()) {
        if (blackStartTime == 0) {
          blackStartTime = millis(); // Start timing
        }
        
        if (millis() - blackStartTime >= 200) { //If all sensors read black after 150 milliseconds, drop the cone
          blackCrossCount++;
          if (blackCrossCount == 1) {
            // Ignore first black area
          } else if (blackCrossCount == 2 && !movingBackward) {
            gripper(GRIPPER_OPEN);
            moveBackward(230, 240);
            backwardStartTime = millis();
            movingBackward = true;
          }
        }
      } else {
        blackStartTime = 0; // Reset timer if not continuously black
        if (movingBackward) {
          if (millis() - backwardStartTime >= 1000) { // Move backward for 1 second
            stopMotors();
            movingBackward = false;
            stopRobot = true; // End loop execution
          } else {
            moveBackward(230, 240);
          }
        } else {
          followLine(5, 2, 240, 240, 5);
          gripper(GRIPPER_CLOSE);  // Ensure gripper remains closed while following line
        }
      }
    }
  }
}

void moveForward(int left, int right) {
  analogWrite(MOTOR_A_IN1_PIN, left);
  analogWrite(MOTOR_B_IN1_PIN, right);
}

void moveBackward(int left, int right) {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_B_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, left);
  analogWrite(MOTOR_B_IN2_PIN, right);
}

void stopMotors() {
  analogWrite(MOTOR_A_IN1_PIN, 0);
  analogWrite(MOTOR_B_IN1_PIN, 0);
  analogWrite(MOTOR_A_IN2_PIN, 0);
  analogWrite(MOTOR_B_IN2_PIN, 0);
}

void gripper(int pulse) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(SERVO_PIN, LOW);
}

void followLine(int leftSensor, int rightSensor, int leftSpeed, int rightSpeed, int time) {
  // Calculating the difference between the left and the right sensor
  int error = analogRead(sensors[leftSensor]) - analogRead(sensors[rightSensor]);

  int derivative = error - lastError; // Calculating how fast the error changes
  lastError = error;

  int correction = (Kp * error) + (Kd * derivative); // Calculating the correction for the speed of the wheels

  // Calculating how much the wheels have to speed up / slow down
  int lSpeed = constrain(baseSpeed - correction, 0, leftSpeed);
  int rSpeed = constrain(baseSpeed + correction, 0, rightSpeed);

  // Limiting the speed of the wheels
  analogWrite(MOTOR_A_IN1_PIN, lSpeed);
  analogWrite(MOTOR_B_IN1_PIN, rSpeed);

  // Adding a short delay for a smoother response
  delay(time);
}

bool allSensorsBlack() {
  int blackCount = 0;
  for (int i = 0; i < 8; i++) {
    if (analogRead(sensors[i]) > 500) {
      blackCount++;
    }
  }

  return (blackCount == 8);
}

long getDistance() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  return pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;
}
