#include <ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#define RESET_RELAY_PIN 3 // Pin for resetting the Relay (OFF)
#define SET_RELAY_PIN 4   // Pin for setting the Relay (ON)
#define RECTIFIER_INPUT 5 // Pin for the Rectifier

ros::NodeHandle nh;

bool isReady() {
    if (digitalRead(RECTIFIER_INPUT) == HIGH) return true; // If the rectifier is ready, return true
    else return false;
}

bool turnOnCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    nh.loginfo("Fog machine is warming up");
    while (!isReady()) {
      nh.spinOnce();
      delay(10);
    } // Wait until the fog machine is ready
    FogTurnOn();
    nh.loginfo("Fog machine to be turned on soon!");
    return true;
}

bool turnOffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    FogTurnOff();
    nh.loginfo("Fog machine to be turned off soon!");
    return true;
}

ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> turnOnServer("/fog_machine/turn_on",
                                                                                     &turnOnCallback);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> turnOffServer("/fog_machine/turn_off",
                                                                                      &turnOffCallback);

void setup() {
    pinMode(RESET_RELAY_PIN, OUTPUT); // Set pin 4 as an output
    pinMode(SET_RELAY_PIN, OUTPUT);   // Set pin 3 as an output
    pinMode(RECTIFIER_INPUT, INPUT);  // Set pin 5 as an inputs

    nh.initNode();
    nh.advertiseService(turnOnServer);
    nh.advertiseService(turnOffServer);
}

// Turn on the fog machine by setting the relay
void FogTurnOn() {
    digitalWrite(RESET_RELAY_PIN, LOW);
    delay(10); // Adding delay to set pin 12 to low
    digitalWrite(SET_RELAY_PIN, HIGH);
}

// Turn off the fog machine by resetting the relay
void FogTurnOff() {
    digitalWrite(SET_RELAY_PIN, LOW);
    delay(10);
    digitalWrite(RESET_RELAY_PIN, HIGH);
}

void loop() {
    nh.spinOnce();
    delay(10);
}