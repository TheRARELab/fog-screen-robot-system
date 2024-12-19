#define SAMPLES 500            // Number of samples per reading
#define ANALOG_PIN A6          // Analog pin for EMF sensor
#define RESET_RELAY_PIN 12     // Pin for resetting the Relay (OFF)
#define SET_RELAY_PIN 13       // Pin for setting the Relay (ON)
#define FREQUENCY 60.0         // Frequency to detect, in Hz
#define THRESHOLD 400.0        // Threshold amplitude to trigger the output
#define POWER_ON_LED 5         // The LED indicates that power is on.
#define IS_WARM_LED 6          // The LED indicates that the fog machine is warm
#define GND_PIN 3               // Pin used as ground
bool PowerOn = false;          // A flag to store the state of the fog machine
bool FogMachineIsHot = false;  // A flag to store the state of the readiness of the fog machine

void setup() {
  pinMode(RESET_RELAY_PIN, OUTPUT);           // Set pin 12 as an output
  pinMode(SET_RELAY_PIN, OUTPUT);           // Set pin 13 as an output
  pinMode(POWER_ON_LED, OUTPUT);           // Set pin 5 as an output
  pinMode(GND_PIN, INPUT);                // Set pin 3 as input
  pinMode(IS_WARM_LED, OUTPUT);           // Set pin 6 as output
  Serial.begin(9600);            // Start serial communication at 9600 baud
}

// Calculates the amplitude of the EMF waves
double get_amplitude() {
  // Used for calculating the amplitude:
  double amplitude, realComponent = 0, imaginaryComponent = 0, phi, t;
  int16_t adcValue;

  for (int i = 0; i < SAMPLES; i++) {
    adcValue = analogRead(ANALOG_PIN) - 512;  // Center reading at 0
    t = micros();                             // Current time in miliseconds
    // Calculate the angle for 60 HZ
    phi = 2.0 * PI * t * FREQUENCY / 1.0e6;
    realComponent += adcValue * cos(phi);  // Calculate the real part of power
    imaginaryComponent += adcValue * sin(phi);  // Calculate the apparent part of power
  }
  return 2 * sqrt((realComponent / SAMPLES) * (realComponent / SAMPLES) + (imaginaryComponent / SAMPLES) * (imaginaryComponent / SAMPLES));
}

// Turn on the fog machine by setting the relay
void FogTurnOn() {
  digitalWrite(POWER_ON_LED, HIGH);
  digitalWrite(SET_RELAY_PIN, HIGH);
  digitalWrite(RESET_RELAY_PIN, LOW);
}

// Turn off the fog machine by reseting the relay
void FogTurnOff() {
  digitalWrite(POWER_ON_LED, LOW);
  digitalWrite(SET_RELAY_PIN, LOW);
  digitalWrite(RESET_RELAY_PIN, HIGH);
}

void loop() {
  // Calculate the amplitude of the EMF waves:
  double amplitude = get_amplitude();
  Serial.println(amplitude);

  if (amplitude > THRESHOLD) {  // Determine if the fog machine is ready
    FogMachineIsHot = true;
    digitalWrite(IS_WARM_LED, HIGH);  // Turn on LED
  } else {
    FogMachineIsHot = false;
    digitalWrite(IS_WARM_LED, LOW);  // Turn off LED
  }

  // Read the input from the serial port:
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the incoming string until newline
    command.trim();                                 // Trim whitespace and newline characters.
    command.toLowerCase();                          // Set all characters to lower case
    if (command == "on" && FogMachineIsHot) {       // Turn on the fog machine if it is ready
      PowerOn = true;
      Serial.println("Fog Machine is ON");
    } else if (command == "on" && !FogMachineIsHot) {  // Wait until the fog machine is ready
      PowerOn = true;
      Serial.println("Power set to on, but fog machine is warming up.");
    } else if (command == "off") {
      Serial.println("Fog Machine is OFF");
      PowerOn = false;
    }
  }
  // Turn on or off the fog machine
  if (PowerOn) {
    FogTurnOn();
  } else {
    FogTurnOff();
  }

  delay(500); // Delay for preventing problems
}