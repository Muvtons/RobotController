#include <RobotController.h>

RobotController robot;

void setup() {
  robot.begin(); // Initialize with default settings
}

void loop() {
  robot.moveForward(1000); // Move forward 1 meter
  delay(2000);             // Wait 2 seconds
  robot.turnRight(90);     // Turn right 90 degrees
  delay(2000);             // Wait 2 seconds
  robot.loop();            // Handle background tasks
}
