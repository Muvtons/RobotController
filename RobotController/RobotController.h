#ifndef RobotController_h
#define RobotController_h

#include <Arduino.h>
#include "driver/ledc.h"

class RobotController {
public:
  // Simplified configuration structure
  struct Config {
    float wheelDiameter = 200.0;    // mm
    float wheelBase = 600.0;        // mm
    float moveSpeed = 65.0;         // %
    float turnSpeed = 50.0;         // %
    float linearCalibration = 0.895;
    float turnCalibration = 0.55;
    float kp = 0.5;                 // PID gains
    float ki = 0.02;
    float kd = 0.15;
    float maxCorrection = 15.0;     // Max PWM correction %
    uint8_t leftEncoderPin = 2;
    uint8_t rightEncoderPin = 3;
    uint8_t leftPWMPin = 5;
    uint8_t rightPWMPin = 6;
    uint8_t leftDirPin = 7;
    uint8_t rightDirPin = 8;
    uint8_t leftBrakePin = 9;
    uint8_t rightBrakePin = 10;
  };

  RobotController();
  void begin(const Config& config = Config());
  void moveForward(float distanceMM);
  void turnLeft(float degrees);
  void turnRight(float degrees);
  void stop();
  void enableSerialControl(bool enable = true);
  void loop(); // Must be called in main loop

private:
  Config _config;
  volatile long _encL = 0;
  volatile long _encR = 0;
  volatile bool _inCriticalTurn = false;
  volatile bool _stopRequested = false;
  String _inputString = "";
  bool _stringComplete = false;
  bool _serialControlEnabled = true;
  float _prevError = 0;
  float _integral = 0;
  unsigned long _lastPIDTime = 0;
  
  // Internal methods
  void _initHardware();
  void _setMotorSpeeds(float left, float right);
  void _stopRobot();
  void _setMotorDirection(bool forward);
  void _getEncoderCounts(long &left, long &right);
  void _resetEncoders();
  long _calculateDistancePulses(float mm);
  long _calculateTurnPulses(float deg);
  float _getTurnSlowdownStart(float deg);
  void _moveStraight(float distanceMM, bool forward);
  void _turn(float degrees, bool rightTurn);
  void _processSerial();
  void _processCommand(String cmd);
  
  // ISR handlers
  static void IRAM_ATTR _isrLeft();
  static void IRAM_ATTR _isrRight();
  static RobotController* _instance;
};

#endif