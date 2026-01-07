#pragma once

#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include <Wire.h>

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;
ButtonA buttonA;
OLED display;
IMU imu;

#include "TurnSensor.h"

class PID {
public:
  PID(float kp, float ki, float kd)
      : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0) {}

  float compute(float setpoint, float measured) {
    float dt = (millis() - last_time) / 1000.0; // sec
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    previous_error = error;
    last_time = millis();
    return kp * error + ki * integral + kd * derivative;
  }

private:
  float kp;
  float ki;
  float kd;
  float previous_error;
  float integral;
  unsigned long last_time;
};

float heading = 0.0;        // deg
float target_heading = 0.0; // deg
float lateral = 0.0;        // cm
float target_lateral = 0.0; // cm

PID heading_pid(18.0, 0.0, 0.025);
PID lateral_pid(2.0, 0.0, 0.1);

void update_heading() {
  turnSensorUpdate();

  int32_t signedAngle = (int32_t)turnAngle;

  heading = (float)signedAngle / (float)turnAngle1;
  if (heading > 180.0)
    heading -= 360.0;
  if (heading < -180.0)
    heading += 360.0;
}

int heading_control() {
  float measured = heading;
  while (target_heading - measured > 180.0f)
    measured += 360.0f;
  while (target_heading - measured < -180.0f)
    measured -= 360.0f;

  float control_signal = heading_pid.compute(target_heading, measured);
  return constrain((int)control_signal, -100, 100);
}

float distance_to_counts(float distance) {
  const float wheel_diameter_cm = 3.2;
  const float counts_per_revolution = 358.3;
  float circumference = 3.14159 * wheel_diameter_cm;
  return (distance / circumference) * counts_per_revolution;
}

int lateral_control() {
  int32_t left_counts = encoders.getCountsLeft();
  int32_t right_counts = encoders.getCountsRight();
  float average_counts = (left_counts + right_counts) / 2.0;
  float target_counts = distance_to_counts(target_lateral);

  int control_signal = lateral_pid.compute(target_counts, average_counts);
  return constrain(control_signal, -200, 200);
}

void apply_controls() {
  int heading_signal = heading_control();
  int lateral_signal = lateral_control();

  int left_speed = lateral_signal - heading_signal;
  int right_speed = lateral_signal + heading_signal;

  motors.setSpeeds(left_speed, right_speed);
}

struct Command {
  enum Type { FORWARD, BACKWARD, LEFT, RIGHT } type;
  float value;    // cm or deg
  float duration; // seconds
};

double time_per_move;
int current_command = 0;

void setup() {
  turnSensorSetup();
  turnSensorReset();

  display.clear();

  int TARGET_TIME = 67; // sec
  time_per_move = (double) TARGET_TIME / ((double) (sizeof(commands) / sizeof(commands[0])));
  time_per_move = constrain(time_per_move, 0.5, 2.5);
}

Command f50 = {Command::FORWARD, 50.0, time_per_move};
Command f35 = {Command::FORWARD, 30.0, time_per_move};
Command b50 = {Command::BACKWARD, 50.0, time_per_move};
Command f25 = {Command::FORWARD, 25.0, time_per_move};
Command b25 = {Command::BACKWARD, 25.0, time_per_move};
Command l = {Command::LEFT, 90.0, time_per_move};
Command r = {Command::RIGHT, 90.0, time_per_move};

Command commands[] = {f50, f50, f50, f50, f25, r, f50, r,   f50, f50, l,
                      f50, l,   f50, r,   f50, l, f50, b50, b50, b50, r,
                      b50, b50, l,   b50, f50, r, f50, f50, l,   b50};

void loop() {
  update_heading();

  display.gotoXY(0, 0);
  display.print(heading);

  apply_controls();

  static unsigned long command_start_time = 0;
  if (current_command < sizeof(commands) / sizeof(commands[0])) {
    Command cmd = commands[current_command];
    if (command_start_time == 0) {
      command_start_time = millis();
      if (cmd.type == Command::FORWARD) {
        target_lateral += cmd.value;
      } else if (cmd.type == Command::BACKWARD) {
        target_lateral -= cmd.value;
      } else if (cmd.type == Command::LEFT) {
        target_heading += cmd.value;
        if (target_heading < -180.0)
          target_heading -= 360.0;
      } else if (cmd.type == Command::RIGHT) {
        target_heading -= cmd.value;
        if (target_heading > 180.0)
          target_heading += 360.0;
      }
    }
    if (millis() - command_start_time >= cmd.duration * 1000) {
      current_command++;

      display.println(heading);
      command_start_time = 0;
    }
  } else {
    motors.setSpeeds(0, 0);
  }

  delay(10);
}