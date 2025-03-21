// Define LED pins
int redLed = 13;
int greenLed = 12;
int yellowLed = 11;

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
}

void loop() {

  // Red light - 3 seconds
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, LOW);
  delay(3000);

  // Green light - 4 seconds
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, HIGH);
  digitalWrite(yellowLed, LOW);
  delay(4000);

  // Yellow light - 1 second
  digitalWrite(redLed, LOW);
  digitalWrite(greenLed, LOW);
  digitalWrite(yellowLed, HIGH);
  delay(1000);
}