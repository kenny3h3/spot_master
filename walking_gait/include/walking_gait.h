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
  TURNING_LEFT = 5
};

class ForwardMotion {
 public:
  STATE_CLASS(ForwardMotion)

  // Mathematical function. f(x) = -sqrt(25 - ((2x) - 2)^2) + 23 + gap.
  float curved(float x, float dist_from_origin, float gap);
  void stabilize_legs();
  void output_coordinates();

  // Used for modes.
  void walk();
  void turn();
  void wake_up();

  float back_leg_gap = 1.0f;
  // Stand-Basis (größer = tieferer Stand)
  float neutral_y = 23.0f;   // vorher effektiv 23.0
  
  // Neutralposition vor/zurück (kleiner = Füße weiter hinten)
  float neutral_x_front = 3.5f;   // vorher 3.5
  float neutral_x_back  = 3.1f;   // optional leicht anders für hinten
 
  Mode mode = SITTING_DOWN;
  smov::TrigonometryState trig = smov::TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &hip_body_distance);
  smov::Vector3 coord1, coord2, coord3, coord4;
  bool leg1_motion_done = true, leg2_motion_done = true, leg3_motion_done = true, leg4_motion_done = true;
  bool has_finished_walk = false, done_once = false, request_to_stop_walk = false, has_finished_turn = false;
  float i1 = 0, i2 = 0, i3 = 0, i4 = 0;

  // Used for reading terminal values.
  struct termios old_chars, new_chars;
};
