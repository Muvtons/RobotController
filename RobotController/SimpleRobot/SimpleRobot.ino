#include <RobotController.h>

RobotController robot;

void setup() {
  robot.begin(); // 1 line: Initialize with default settings
}

void loop() {
  robot.moveForward(1000); // 2 lines: Move forward 1 meter
  robot.turnRight(90);     // 3 lines: Turn right 90 degrees
  robot.loop();            // 4 lines: Handle background tasks
}