#include "RobotController.h"
#include <Arduino.h>

RobotController* RobotController::_instance = nullptr;

RobotController::RobotController() {
  _instance = this;
}

void RobotController::begin() {
  Config defaultConfig;
  begin(defaultConfig);
}

void RobotController::begin(const Config& config) {
  _config = config;
  _initHardware();
  Serial.begin(115200);
  delay(500);
  Serial.println("\n🤖 Robot Controller Ready!");
  Serial.println("Commands: move_forward, turn_left 90, turn_right 90, STOP");
}

void RobotController::_initHardware() {
  pinMode(_config.leftEncoderPin, INPUT_PULLUP);
  pinMode(_config.rightEncoderPin, INPUT_PULLUP);
  pinMode(_config.leftDirPin, OUTPUT);
  pinMode(_config.rightDirPin, OUTPUT);
  pinMode(_config.leftBrakePin, OUTPUT);
  pinMode(_config.rightBrakePin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(_config.leftEncoderPin), _isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(_config.rightEncoderPin), _isrRight, RISING);

  ledc_timer_config_t timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 2000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  ledc_channel_config_t ch = {
    .gpio_num = _config.leftPWMPin, .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0
  };
  ledc_channel_config(&ch);

  ch.gpio_num = _config.rightPWMPin;
  ch.channel = LEDC_CHANNEL_1;
  ledc_channel_config(&ch);

  _stopRobot();
  _resetEncoders();
}

void RobotController::_setMotorSpeeds(float leftSpeed, float rightSpeed) {
  leftSpeed = constrain(leftSpeed, 0, 100);
  rightSpeed = constrain(rightSpeed, 0, 100);
  
  uint32_t pwmL = (uint32_t)(leftSpeed / 100.0f * 4095);
  uint32_t pwmR = (uint32_t)(rightSpeed / 100.0f * 4095);
  
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwmL);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwmR);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void RobotController::_stopRobot() {
  digitalWrite(_config.leftBrakePin, HIGH);
  digitalWrite(_config.rightBrakePin, HIGH);
  _setMotorSpeeds(0, 0);
}

void RobotController::_setMotorDirection(bool forward) {
  digitalWrite(_config.leftBrakePin, LOW);
  digitalWrite(_config.rightBrakePin, LOW);
  digitalWrite(_config.leftDirPin, forward ? HIGH : LOW);
  digitalWrite(_config.rightDirPin, forward ? HIGH : LOW);
}

void RobotController::_getEncoderCounts(long &left, long &right) {
  noInterrupts();
  left = _encL;
  right = _encR;
  interrupts();
}

void RobotController::_resetEncoders() {
  noInterrupts();
  _encL = 0;
  _encR = 0;
  interrupts();
}

long RobotController::_calculateDistancePulses(float mm) {
  float circ = PI * _config.wheelDiameter;
  return (long)((mm / circ) * (24 * 30) * _config.linearCalibration);
}

long RobotController::_calculateTurnPulses(float degrees) {
  float arc = (PI * _config.wheelBase) * (degrees / 360.0f);
  return (long)((arc / (PI * _config.wheelDiameter)) * (24 * 30) * _config.turnCalibration);
}

float RobotController::_getTurnSlowdownStart(float deg) {
  if (deg <= 45) return 0.85;
  if (deg <= 90) return 0.35;
  if (deg <= 180) return 0.65;
  return 0.60;
}

void RobotController::_moveStraight(float distanceMM, bool forward) {
  if (_inCriticalTurn) {
    Serial.println(">> Move blocked (turn in progress)");
    return;
  }
  
  long target = _calculateDistancePulses(distanceMM);
  long slowdown_start = target * 0.80;
  
  _resetEncoders();
  _prevError = 0;
  _integral = 0;
  _lastPIDTime = millis();

  _setMotorDirection(forward);
  _setMotorSpeeds(_config.moveSpeed, _config.moveSpeed);

  Serial.printf("Moving %.0fmm %s (target: %ld pulses)\n", 
                distanceMM, forward ? "FORWARD" : "BACKWARD", target);

  unsigned long lastMonitor = millis();
  unsigned long lastPIDCalc = millis();
  bool slowdownPhase = false;

  while (1) {
    if (_stopRequested) {
      Serial.println(">> STOP requested during straight move");
      break;
    }
    
    long l, r;
    _getEncoderCounts(l, r);
    long avg = (l + r) / 2;
    
    if (avg >= target) break;

    if (avg >= slowdown_start && !slowdownPhase) {
      slowdownPhase = true;
    }
    
    if (slowdownPhase) {
      float slowdown_progress = (float)(avg - slowdown_start) / max(1L, target - slowdown_start);
      slowdown_progress = constrain(slowdown_progress, 0.0f, 1.0f);
      float current_speed = _config.moveSpeed - (_config.moveSpeed - 5.0) * slowdown_progress;
      _setMotorSpeeds(current_speed, current_speed);
      lastPIDCalc = millis();
      continue;
    }

    if (millis() - lastPIDCalc >= 50) {
      long error = l - r;
      float dt = (millis() - lastPIDCalc) / 1000.0f;
      _integral += error * dt;
      
      if (_integral > 100) _integral = 100;
      if (_integral < -100) _integral = -100;
      
      float derivative = (error - _prevError) / dt;
      float correction = _config.kp * error + _config.ki * _integral + _config.kd * derivative;
      correction = constrain(correction, -_config.maxCorrection, _config.maxCorrection);
      
      float leftSpeed = _config.moveSpeed;
      float rightSpeed = _config.moveSpeed;
      
      if (error > 0) {
        leftSpeed -= abs(correction);
      } else {
        rightSpeed -= abs(correction);
      }
      
      float minSpeed = _config.moveSpeed - _config.maxCorrection;
      leftSpeed = max(leftSpeed, minSpeed);
      rightSpeed = max(rightSpeed, minSpeed);
      
      _setMotorSpeeds(leftSpeed, rightSpeed);
      _prevError = error;
      lastPIDCalc = millis();
    }

    if (millis() - lastMonitor >= 100) {
      long error = l - r;
      float progress = (float)avg / target * 100;
      const char* phase = slowdownPhase ? "SLOWDOWN" : "CRUISING";
      Serial.printf("Progress: %.1f%% | L=%ld R=%ld | Error=%ld | %s\n",
                    progress, l, r, error, phase);
      lastMonitor = millis();
    }
    
    delay(1);
  }

  _stopRobot();
  delay(300);
  
  long finalL, finalR;
  _getEncoderCounts(finalL, finalR);
  long finalError = finalL - finalR;
  long actualAvg = (finalL + finalR) / 2;
  float overshootPercent = ((float)actualAvg / target - 1.0f) * 100.0f;
  
  Serial.printf("  ✓ Complete: L=%ld R=%ld | Avg=%ld (target=%ld) | Error=%ld (%.1fmm)\n", 
                finalL, finalR, actualAvg, target,
                finalError, finalError * (PI * _config.wheelDiameter / (24 * 30)));
  Serial.printf("  Overshoot: %.1f%%\n\n", overshootPercent);
}

void RobotController::_turn(float degrees, bool rightTurn) {
  if (_inCriticalTurn) {
    Serial.println(">> Turn command ignored (another turn in progress)");
    return;
  }
  
  _inCriticalTurn = true;
  _stopRequested = false;

  long target = _calculateTurnPulses(abs(degrees));
  float slowdown_percent = _getTurnSlowdownStart(degrees);
  long slowdown_start = target * slowdown_percent;

  _resetEncoders();
  _prevError = 0;
  _integral = 0;
  _lastPIDTime = millis();

  digitalWrite(_config.leftBrakePin, LOW);
  digitalWrite(_config.rightBrakePin, LOW);
  digitalWrite(_config.leftDirPin, rightTurn ? HIGH : LOW);
  digitalWrite(_config.rightDirPin, rightTurn ? LOW : HIGH);
  _setMotorSpeeds(_config.turnSpeed, _config.turnSpeed);

  Serial.printf("Turning %.0f° %s (target: %ld pulses, slowdown at: %d%%)\n", 
                degrees, rightTurn ? "RIGHT" : "LEFT", target, (int)(slowdown_percent * 100));

  unsigned long lastMonitor = millis();
  unsigned long lastPIDCalc = millis();
  bool slowdownPhase = false;
  bool interrupted = false;

  while (1) {
    if (_stopRequested) {
      Serial.println(">> STOP received during turn - aborting immediately");
      interrupted = true;
      break;
    }

    long l, r;
    _getEncoderCounts(l, r);
    l = abs(l);
    r = abs(r);
    long avg = (l + r) / 2;
    
    if (avg >= target) break;

    if (avg >= slowdown_start && !slowdownPhase) {
      slowdownPhase = true;
      Serial.printf("  >> SLOWDOWN ACTIVATED (%d%%) <<\n", (int)(slowdown_percent * 100));
    }

    if (millis() - lastPIDCalc >= 50) {
      long error = l - r;
      float dt = (millis() - lastPIDCalc) / 1000.0f;
      _integral += error * dt;
      
      if (_integral > 50) _integral = 50;
      if (_integral < -50) _integral = -50;
      
      float derivative = (error - _prevError) / dt;
      float correction = _config.kp * error + _config.ki * _integral + _config.kd * derivative;
      correction = constrain(correction, -_config.maxCorrection, _config.maxCorrection);
      
      float leftSpeed = _config.turnSpeed;
      float rightSpeed = _config.turnSpeed;
      
      if (error > 0) {
        leftSpeed -= abs(correction);
      } else {
        rightSpeed -= abs(correction);
      }
      
      float minSpeed = _config.turnSpeed - _config.maxCorrection;
      leftSpeed = max(leftSpeed, minSpeed);
      rightSpeed = max(rightSpeed, minSpeed);
      
      _setMotorSpeeds(leftSpeed, rightSpeed);
      _prevError = error;
      lastPIDCalc = millis();
    }

    if (slowdownPhase) {
      float slowdown_progress = (float)(avg - slowdown_start) / max(1L, target - slowdown_start);
      slowdown_progress = constrain(slowdown_progress, 0.0f, 1.0f);
      float current_speed = _config.turnSpeed - (_config.turnSpeed - 5.0) * (slowdown_progress * slowdown_progress);
      _setMotorSpeeds(current_speed, current_speed);
      lastPIDCalc = millis();
    }

    if (millis() - lastMonitor >= 100) {
      long error = l - r;
      float progress = (float)avg / target * 100;
      const char* phase = slowdownPhase ? "SLOWDOWN" : "CRUISING";
      Serial.printf("  Progress: %.1f%% | L=%ld R=%ld | Error=%ld | %s\n",
                    progress, l, r, error, phase);
      lastMonitor = millis();
    }
  }

  _stopRobot();
  delay(300);
  
  long finalL, finalR;
  _getEncoderCounts(finalL, finalR);
  long actualAvg = (abs(finalL) + abs(finalR)) / 2;
  float overshootPercent = ((float)actualAvg / target - 1.0f) * 100.0f;
  
  if (interrupted) {
    Serial.printf("  ✗ TURN ABORTED by STOP: L=%ld R=%ld | Avg=%ld (target=%ld)\n", 
                  abs(finalL), abs(finalR), actualAvg, target);
  } else {
    Serial.printf("  ✓ Complete: L=%ld R=%ld | Avg=%ld (target=%ld)\n", 
                  abs(finalL), abs(finalR), actualAvg, target);
    Serial.printf("  Overshoot: %.1f%%\n\n", overshootPercent);
  }
  
  _inCriticalTurn = false;
  _stopRequested = false;
}

void RobotController::_processSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (_inCriticalTurn) {
      if (c == '\n' || c == '\r') {
        _inputString.trim();
        if (_inputString.equalsIgnoreCase("STOP")) {
          _stopRequested = true;
          Serial.println(">> STOP registered during turn");
        }
        _inputString = "";
      } else if (_inputString.length() < 20) {
        _inputString += c;
      }
      continue;
    }

    if (c == '\n' || c == '\r') {
      _stringComplete = true;
    } else {
      _inputString += c;
    }
  }
}

void RobotController::_processCommand(String cmd) {
  cmd.trim();
  if (cmd == "") return;

  if (_inCriticalTurn) {
    Serial.println(">> Command ignored (turn in progress)");
    return;
  }

  Serial.print("Executing: [");
  Serial.print(cmd);
  Serial.println("]");

  if (cmd.equalsIgnoreCase("move_forward")) {
    _moveStraight(1000.0, true);
    return;
  }

  if (cmd.equalsIgnoreCase("turnaround")) {
    _turn(360.0, true);
    return;
  }

  if (cmd.startsWith("turn_left ")) {
    String angleStr = cmd.substring(9);
    int angle = angleStr.toInt();
    if (angle > 0 && angle <= 360) {
      _turn(angle, false);
      return;
    }
  }

  if (cmd.startsWith("turn_right ")) {
    String angleStr = cmd.substring(10);
    int angle = angleStr.toInt();
    if (angle > 0 && angle <= 360) {
      _turn(angle, true);
      return;
    }
  }

  if (cmd.equalsIgnoreCase("STOP")) {
    _stopRequested = true;
    Serial.println(">> STOPPED by command");
    return;
  }

  Serial.println(">> Unknown command");
}

void IRAM_ATTR RobotController::_isrLeft() {
  if (_instance) _instance->_encL++;
}

void IRAM_ATTR RobotController::_isrRight() {
  if (_instance) _instance->_encR++;
}

void RobotController::moveForward(float distanceMM) {
  _moveStraight(distanceMM, true);
}

void RobotController::turnLeft(float degrees) {
  _turn(degrees, false);
}

void RobotController::turnRight(float degrees) {
  _turn(degrees, true);
}

void RobotController::stop() {
  _stopRequested = true;
}

void RobotController::enableSerialControl(bool enable) {
  _serialControlEnabled = enable;
}

void RobotController::loop() {
  if (_serialControlEnabled && (_stringComplete || Serial.available())) {
    if (_stringComplete) {
      _processCommand(_inputString);
      _inputString = "";
      _stringComplete = false;
    }
    _processSerial();
  }
  
  static unsigned long lastStatus = 0;
  if (!_inCriticalTurn && millis() - lastStatus > 2000) {
    lastStatus = millis();
  }
}
