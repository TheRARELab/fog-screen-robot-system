#include <ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#define RESET_RELAY_PIN 7  // Pin for resetting the Relay (OFF)
#define SET_RELAY_PIN 6    // Pin for setting the Relay (ON)
#define RECTIFIER_INPUT 5  // Pin for the Rectifier
#define GROUND_PIN 9       // Pin for Ground

ros::NodeHandle nh;

inline bool isReady() {
  if (digitalRead(RECTIFIER_INPUT) == HIGH) return true;  // If the rectifier is ready, return true
  else return false;
}

void turnOnCallback(const std_srvs::Empty::Request &req, std_srvs::Trigger::Response &res) {
  bool first_iteration = true;
  while (!isReady()) {
    if (first_iteration) {  // Tell the client that the fog machine is warming up only one time
      nh.loginfo("Fog machine is warming up");
      first_iteration = false;
    }
  }
  FogTurnOn();
  res.success = true;  // Tell the client that the function call was successful.
  nh.loginfo("Fog machine is on");
}

void turnOffCallback(const std_srvs::Empty::Request &req, std_srvs::Trigger::Response &res) {
  FogTurnOff();
  res.success = true;  // Tell the client that the function call was successful.
  nh.loginfo("Fog machine to be turned off soon!");
}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Trigger::Response> turnOnServer("/fog_machine/turn_on", &turnOnCallback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Trigger::Response> turnOffServer("/fog_machine/turn_off", &turnOffCallback);


void setup() {
  pinMode(RESET_RELAY_PIN, OUTPUT);  // Set pin 4 as an output
  pinMode(SET_RELAY_PIN, OUTPUT);    // Set pin 3 as an output
  pinMode(RECTIFIER_INPUT, INPUT);   // Set pin 5 as an inputs
  pinMode(GROUND_PIN, OUTPUT);       // Set pin 9 as output

  digitalWrite(GROUND_PIN, LOW);       // The GROUND_PIN is used as ground
  digitalWrite(RESET_RELAY_PIN, LOW);  // Set the PIN to low when it starts
  digitalWrite(SET_RELAY_PIN, LOW);

  nh.initNode();
  nh.advertiseService(turnOnServer);
  nh.advertiseService(turnOffServer);
}

// Turn on the fog machine by setting the relay
void FogTurnOn() {
  digitalWrite(SET_RELAY_PIN, HIGH);
  delay(200);                        // Wait 200 milliseconds for the relay to change state
  digitalWrite(SET_RELAY_PIN, LOW);  // We save energy by turning off the pin
}

// Turn off the fog machine by resetting the relay
void FogTurnOff() {
  digitalWrite(RESET_RELAY_PIN, HIGH);
  delay(200);
  digitalWrite(RESET_RELAY_PIN, LOW);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
