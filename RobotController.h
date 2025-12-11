#ifndef RobotController_h
#define RobotController_h

#include <Arduino.h>
#include "driver/ledc.h"

class RobotController {
public:
  // Configuration structure (must be fully defined before use)
  struct Config {
    float wheelDiameter;
    float wheelBase;
    float moveSpeed;
    float turnSpeed;
    float linearCalibration;
    float turnCalibration;
    float kp;
    float ki;
    float kd;
    float maxCorrection;
    uint8_t leftEncoderPin;
    uint8_t rightEncoderPin;
    uint8_t leftPWMPin;
    uint8_t rightPWMPin;
    uint8_t leftDirPin;
    uint8_t rightDirPin;
    uint8_t leftBrakePin;
    uint8_t rightBrakePin;
    
    // Constructor with default values
    Config() : 
      wheelDiameter(200.0),
      wheelBase(600.0),
      moveSpeed(65.0),
      turnSpeed(50.0),
      linearCalibration(0.895),
      turnCalibration(0.55),
      kp(0.5),
      ki(0.02),
      kd(0.15),
      maxCorrection(15.0),
      leftEncoderPin(2),
      rightEncoderPin(3),
      leftPWMPin(5),
      rightPWMPin(6),
      leftDirPin(7),
      rightDirPin(8),
      leftBrakePin(9),
      rightBrakePin(10) {}
  };

  RobotController();
  void begin(); // Default configuration
  void begin(const Config& config); // Custom configuration
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
