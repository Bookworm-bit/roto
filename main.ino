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

const float TILE_WIDTH = 50.0;
const float TRACK_WIDTH = 8.5;
const float MAX_VELOCITY = 20.0; // cm/s
const float ARC_RADIUS = TILE_WIDTH;   // cm - turn radius to center of robot

class PID {
public:
  PID(float kp, float ki, float kd)
      : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0),
        last_time(millis()) {}

  float compute(float setpoint, float measured) {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    if (dt <= 0)
      dt = 0.001;

    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;

    previous_error = error;
    last_time = now;

    return kp * error + ki * integral + kd * derivative;
  }

private:
  float kp, ki, kd;
  float previous_error;
  float integral;
  unsigned long last_time;
};

float heading = 0.0;        // deg
float target_heading = 0.0; // deg

float target_lateral = 0.0; // cm
float lateral = 0.0; // cm

// Arc turn state
bool arc_turn_active = false;
float arc_turn_direction = 0.0; // 1.0 = left turn, -1.0 = right turn
float arc_target_distance = 0.0; // distance outer wheel must travel
float arc_position = 0.0; // distance outer wheel has traveled
float arc_speed_ratio = 0.0; // inner_speed / outer_speed
float arc_start_heading = 0.0; // heading when arc started
float arc_target_angle = 0.0; // total degrees to turn
unsigned long arc_last_time = 0;
int32_t arc_last_counts = 0;
float arc_last_pos = 0.0;
unsigned long arc_vel_last_time = 0;

// Spin turn state
bool spin_turn_active = false;
float spin_turn_direction = 0.0; // 1.0 = left, -1.0 = right
float spin_target_distance = 0.0; // distance each wheel must travel
float spin_position = 0.0;

float velocity = 0.0;        // cm/s
float target_velocity = 0.0; // cm/s

// Velocity tracking state
unsigned long vel_last_time = 0;
int32_t vel_last_counts = 0;

const float MAX_ACCEL = 15.0;      // cm/s^2
const float MAX_DECEL = 15.0;      // cm/s^2

PID heading_pid(18.0, 0.0, 0.025);
PID velocity_pid(5.0, 0.0, 0.0);

void update_heading() {
  turnSensorUpdate();

  int32_t signedAngle = (int32_t)turnAngle;

  heading = (float)signedAngle / (float)turnAngle1;
  if (heading > 180.0)
    heading -= 360.0;
  if (heading < -180.0)
    heading += 360.0;
}

void update_velocity() {
  unsigned long now = millis();
  float dt = (now - vel_last_time) / 1000.0;
  if (dt <= 0) dt = 0.001;

  int32_t left_counts = encoders.getCountsLeft();
  int32_t right_counts = encoders.getCountsRight();
  int32_t average_counts = (left_counts + right_counts) / 2;

  int32_t delta_counts = average_counts - vel_last_counts;
  const float wheel_diameter_cm = 3.2;
  const float counts_per_revolution = 358.3;
  float circumference = 3.14159 * wheel_diameter_cm;
  float distance_cm = (delta_counts / counts_per_revolution) * circumference;

  velocity = distance_cm / dt;
  lateral += distance_cm;

  vel_last_counts = average_counts;
  vel_last_time = now;
}

void update() {
  update_velocity();
  update_heading();
}

void update_arc_position() {
  unsigned long now = millis();
  
  int32_t left_counts = encoders.getCountsLeft();
  int32_t right_counts = encoders.getCountsRight();
  
  int32_t outer_counts;
  if (arc_turn_direction > 0) {
    outer_counts = right_counts;
  } else {
    outer_counts = left_counts;
  }
  
  int32_t delta_counts = outer_counts - arc_last_counts;
  const float wheel_diameter_cm = 3.2;
  const float counts_per_revolution = 358.3;
  float circumference = 3.14159 * wheel_diameter_cm;
  float distance_cm = (delta_counts / counts_per_revolution) * circumference;
  
  arc_position += distance_cm;
  
  arc_last_counts = outer_counts;
  arc_last_time = now;
}

void reset_arc_state() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  arc_position = 0.0;
  arc_last_counts = 0;
  arc_last_time = millis();
  arc_last_pos = 0.0;
  arc_vel_last_time = millis();
}

void apply_arc_controls() {
  update_arc_position();
  update_heading();

  float target_vel = compute_trapezoidal_velocity(arc_position, arc_target_distance);

  unsigned long now = millis();
  float dt = (now - arc_vel_last_time) / 1000.0;
  if (dt <= 0) dt = 0.001;
  
  float outer_velocity = (arc_position - arc_last_pos) / dt;
  arc_last_pos = arc_position;
  arc_vel_last_time = now;
  
  float control_signal = velocity_pid.compute(target_vel, outer_velocity);
  float feedforward = target_vel * 20.0;
  int base_speed = constrain((int)(control_signal + feedforward), -200, 200);

  // Calculate expected heading based on arc progress
  float progress = arc_position / arc_target_distance; // 0 to 1
  float expected_heading = arc_start_heading + (arc_turn_direction * arc_target_angle * progress);
  
  // Normalize expected heading
  while (expected_heading > 180.0) expected_heading -= 360.0;
  while (expected_heading < -180.0) expected_heading += 360.0;
  
  // Calculate heading error and correction
  float heading_error = expected_heading - heading;
  while (heading_error > 180.0) heading_error -= 360.0;
  while (heading_error < -180.0) heading_error += 360.0;
  
  // Apply heading correction (positive error = need to turn more left)
  int heading_correction = (int)(heading_error * 3.0); // Tune this gain
  heading_correction = constrain(heading_correction, -50, 50);
  
  int outer_speed = base_speed;
  int inner_speed = (int)(base_speed * arc_speed_ratio);
  
  // Adjust speeds based on heading error
  if (arc_turn_direction > 0) {
    // Left turn: if we need to turn more left, slow inner / speed outer
    motors.setSpeeds(inner_speed - heading_correction, outer_speed + heading_correction);
  } else {
    // Right turn: if we need to turn more right (negative error), adjust opposite
    motors.setSpeeds(outer_speed - heading_correction, inner_speed + heading_correction);
  }
}

bool arc_turn_complete() {
  // Check if heading is close to target
  float error = target_heading - heading;
  // Normalize to -180 to 180
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  
  return abs(error) <= 3.0; // Within 3 degrees of target
}

const float MIN_VELOCITY = 3.0;

float compute_trapezoidal_velocity(float position, float total_distance) {
  float direction = (total_distance >= 0) ? 1.0 : -1.0;
  float abs_distance = abs(total_distance);
  float abs_position = abs(position);
  
  if (abs_distance < 0.1) return 0.0;

  float accel_distance = (MAX_VELOCITY * MAX_VELOCITY) / (2.0 * MAX_ACCEL);
  float decel_distance = (MAX_VELOCITY * MAX_VELOCITY) / (2.0 * MAX_DECEL);
  
  float remaining = abs_distance - abs_position;
  
  if (remaining <= 0.5) return 0.0;

  float vel;
  bool in_accel_phase = false;
  
  if (accel_distance + decel_distance >= abs_distance) {
    float peak_distance = abs_distance * (MAX_DECEL / (MAX_ACCEL + MAX_DECEL));
    
    if (abs_position < peak_distance) {
      vel = sqrt(2.0 * MAX_ACCEL * abs_position);
      in_accel_phase = true;
    } else {
      vel = sqrt(2.0 * MAX_DECEL * remaining);
    }
  } else {
    if (abs_position < accel_distance) {
      vel = sqrt(2.0 * MAX_ACCEL * abs_position);
      in_accel_phase = true;
    } else if (remaining < decel_distance) {
      vel = sqrt(2.0 * MAX_DECEL * remaining);
    } else {
      vel = MAX_VELOCITY;
    }
  }

  if (in_accel_phase && vel < MIN_VELOCITY) {
    vel = MIN_VELOCITY;
  }
  
  return direction * vel;
}

int velocity_control() {
  target_velocity = compute_trapezoidal_velocity(lateral, target_lateral);

  target_velocity = constrain(target_velocity, -MAX_VELOCITY, MAX_VELOCITY);

  float control_signal = velocity_pid.compute(target_velocity, velocity);
  float feedforward = target_velocity * 20.0;
  
  return constrain((int)(control_signal + feedforward), -200, 200);
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

void apply_controls() {
  int heading_signal = heading_control();
  int velocity_signal = velocity_control();

  int left_speed = velocity_signal - heading_signal;
  int right_speed = velocity_signal + heading_signal;

  motors.setSpeeds(left_speed, right_speed);
}

void stop_motors() { motors.setSpeeds(0, 0); }

struct Command {
  enum Type { FORWARD, BACKWARD, LEFT, RIGHT, SPIN_LEFT, SPIN_RIGHT } type;
  float value;    // cm or deg
  float duration; // seconds
};

double time_per_move;
int current_command = 0;

#define FF {Command::FORWARD, TILE_WIDTH, 0}
#define BF {Command::BACKWARD, TILE_WIDTH, 0}
#define FFB {Command::FORWARD, TILE_WIDTH * 52.0 / 50.0, 0}
#define BFB {Command::BACKWARD, TILE_WIDTH * 52.0 / 50.0, 0}
#define FH {Command::FORWARD, TILE_WIDTH / 2.0, 0}
#define BH {Command::BACKWARD, TILE_WIDTH / 2.0, 0}
#define FHB {Command::FORWARD, (TILE_WIDTH / 2.0) * 52.0 / 10.0, 0}
#define BHB {Command::BACKWARD, (TILE_WIDTH / 2.0) * 52.0 / 10.0, 0}
#define L90 {Command::LEFT, 90.0, 0}
#define R90 {Command::RIGHT, 90.0, 0}

Command commands[] = {FF, L90};
const int NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

void setup() {
  turnSensorSetup();
  turnSensorReset();

  display.clear();

  int TARGET_TIME = 2; // sec
  time_per_move = (double)TARGET_TIME / (double)NUM_COMMANDS;
  time_per_move = constrain(time_per_move, 1.0, 2.5);

  for (int i = 0; i < NUM_COMMANDS; i++) {
    commands[i].duration = time_per_move;
  }
}

bool direction = false;

void reset_encoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  lateral = 0.0;
  vel_last_counts = 0;
  vel_last_time = millis();
}


void loop() {
  update();
  
  if (arc_turn_active) {
    apply_arc_controls();
  } else {
    apply_controls();
  }

  display.gotoXY(0, 0);
  display.print(heading);

  // Serial.print("velocity: ");
  // Serial.println(velocity);

  // Serial.print("target_velocity: ");
  // Serial.println(target_velocity);

  static unsigned long command_start_time = 0;
  if (current_command < NUM_COMMANDS) {
    Command cmd = commands[current_command];
    //Serial.println("balls");
    if (command_start_time == 0) {
      command_start_time = millis();
      if (cmd.type == Command::FORWARD) {
        reset_encoders();
        target_lateral = cmd.value;
      } else if (cmd.type == Command::BACKWARD) {
        reset_encoders();
        target_lateral = -cmd.value;
        Serial.println(target_lateral);
      } else if (cmd.type == Command::LEFT) {
        reset_arc_state();
        arc_turn_active = true;
        arc_turn_direction = 1.0;
        arc_start_heading = heading;
        arc_target_angle = cmd.value;
        float outer_radius = ARC_RADIUS + TRACK_WIDTH / 2.0;
        float inner_radius = ARC_RADIUS - TRACK_WIDTH / 2.0;
        arc_speed_ratio = inner_radius / outer_radius;
        arc_target_distance = (cmd.value * 3.14159 / 180.0) * outer_radius;
        target_heading += cmd.value;
        if (target_heading > 180.0)
          target_heading -= 360.0;
        if (target_heading < -180.0)
          target_heading += 360.0;
      } else if (cmd.type == Command::RIGHT) {
        reset_arc_state();
        arc_turn_active = true;
        arc_turn_direction = -1.0;
        arc_start_heading = heading;
        arc_target_angle = cmd.value;
        float outer_radius = ARC_RADIUS + TRACK_WIDTH / 2.0;
        float inner_radius = ARC_RADIUS - TRACK_WIDTH / 2.0;
        arc_speed_ratio = inner_radius / outer_radius;
        arc_target_distance = (cmd.value * 3.14159 / 180.0) * outer_radius;
        target_heading -= cmd.value;
        if (target_heading > 180.0)
          target_heading -= 360.0;
        if (target_heading < -180.0)
          target_heading += 360.0;
      }
    }

    bool move_complete = false;
    if (arc_turn_active) {
      move_complete = arc_turn_complete();
    } else {
      move_complete = (millis() - command_start_time >= cmd.duration * 1000);
    }

    if (move_complete) {
      arc_turn_active = false;
      current_command++;

      display.println(heading);
      command_start_time = 0;
    }
  } else {
    stop_motors();
  }
  delay(10);
}