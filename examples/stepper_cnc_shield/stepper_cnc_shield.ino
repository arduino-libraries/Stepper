// From the pinout diagram

// X: Step = 2, Direction = 5
// Y: Step = 3, Direction = 6
// Z: Step = 4, Direction = 7

byte enablePin = 8;
byte directionPin = 5;
byte stepPin = 2;
int numberOfSteps = 400;
int pulseWidthMicros = 8;  // microseconds
int millisBetweenSteps = 8; // milliseconds

long previousMillis = 0;
bool forward = true;
int stepCount = 0;
bool enabled = true;

/**********************
   void setup() - Initialisations
 ***********************/
void setup() {
  
  //  Setup
  
  Serial.begin( 9600 );
  Serial.println("Starting StepperTest");

  delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Initialise the step count

  int stepCount = 0;

}

void loop() {

  // Avoid using delay as this seems to break the servo
  
  unsigned long currentMillis = millis();

  if (enabled == true) {
    if (currentMillis - previousMillis > millisBetweenSteps) {
      previousMillis = currentMillis;

      if (forward == true) {
        stepCount = stepCount + 1;
        if (stepCount > numberOfSteps) {
          forward = false;
        }
        digitalWrite(directionPin, HIGH);
      }
      else
      {
        stepCount = stepCount - 1;
        if (stepCount < 1) {
          enabled = false;
          digitalWrite(enablePin, HIGH);
        }
        digitalWrite(directionPin, LOW);
      }

      digitalWrite(stepPin, HIGH);
      //delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
      digitalWrite(stepPin, LOW);
      
    }
  }
}
