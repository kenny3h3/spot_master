#pragma once

#include <math.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include "smov/executable.h"
#include "smov/base.h"
#include "smov/trigonometry.h"

enum Mode {
  SITTING_DOWN = 0,
  STANDING = 1,
  WAKING_UP = 2,
  WALKING = 3,
  TURNING_RIGHT = 4,
  TURNING_LEFT = 5,
  CURVING_LEFT = 6,
  CURVING_RIGHT = 7
};

class MotionControl {
 public:
  STATE_CLASS(MotionControl)

  float curved(float x, float dist_from_origin, float gap);
  void stabilize_legs();
  void output_coordinates();
  void walk();
  void turn();
  void curve(bool is_left);
  void wake_up();

  float back_leg_gap = 0.0f;
  float neutral_y = 22.5f;
  float neutral_x_front = 3.5f;
  float neutral_x_back = 3.7f;
 
  Mode mode = SITTING_DOWN;
  Mode pending_mode = STANDING;
  bool has_pending = false;
  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &hip_body_distance);
  smov::Vector3 coord1, coord2, coord3, coord4;
  bool leg1_motion_done = true, leg2_motion_done = true, leg3_motion_done = true, leg4_motion_done = true;
  bool has_finished_walk = false, done_once = false, request_to_stop_walk = false, has_finished_turn = false;
  float i1 = 0, i2 = 0, i3 = 0, i4 = 0;

  struct termios old_chars, new_chars;
};