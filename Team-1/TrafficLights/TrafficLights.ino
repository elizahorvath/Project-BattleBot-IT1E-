//defining the pins
const int LED_RED_PIN = 13;
const int LED_YELLOW_PIN = 12;
const int LED_GREEN_PIN = 11;

const int BUTTON_1_PIN = 5;

void setup() {
  //setting the input/output of the pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
}

void loop() {
  //turn off all the lights before turning only the red light on
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);

  //the default color of the traffic light is red
  digitalWrite(LED_RED_PIN, LOW);

  //the state of the buttons is nul
  bool button1State = 0;

  //reading the state of the button pin
  button1State = digitalRead(BUTTON_1_PIN);

  //if the button is clicked and the red light is already turned on, run the trafficLight() function
  if(button1State == LOW && digitalRead(LED_RED_PIN == LOW)) {
    trafficLight();
  } 
}

void trafficLight() {
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);

  delay(5000);

  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, LOW);

  delay(2000); 

  digitalWrite(LED_YELLOW_PIN, HIGH);
  digitalWrite(LED_RED_PIN, LOW);
}