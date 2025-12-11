#include <RobotController.h>

RobotController robot;

void setup() {
  // Initialize robot on Core 0
  robot.begin();
  
  // Create and start Core 1 task
  xTaskCreatePinnedToCore(
    core1Task,          // Function to run
    "Core1Task",        // Task name
    4096,               // Stack size (bytes)
    NULL,               // Task parameters
    1,                  // Priority (higher number = higher priority)
    NULL,               // Task handle
    1                   // Core ID (Core 1)
  );
  
  robot.safePrintln("\n✅ Multi-Core System Initialized!");
  robot.safePrintf("Core 0 ID: %d\n", xPortGetCoreID());
}

void loop() {
  // Core 0 runs robot control
  robot.run();
  
  // Small delay to prevent watchdog trigger
  delay(1);
}
