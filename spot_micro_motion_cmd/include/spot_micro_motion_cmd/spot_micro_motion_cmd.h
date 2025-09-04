#pragma once

#include <math.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <i2cpwm_board_msgs/msg/servo_array.hpp>
#include <smov_msgs/msg/states_servos.hpp>
#include "smov/trigonometry.h"  <!-- Nutzt smov_lib statt spot_micro_kinematics -->

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

class SpotMicroMotionCmd {
 public:
  STATE_CLASS(SpotMicroMotionCmd)

  SpotMicroMotionCmd();
  ~SpotMicroMotionCmd();

  void stabilize_legs();
  void output_coordinates();
  void walk();
  void turn();
  void curve(bool is_left);
  void wake_up();
  void on_start();
  void on_loop();
  void on_quit();

  float curved(float x, float dist_from_origin, float gap);
  void set_servo_positions(const smov::Vector3& coords);

  Mode mode = SITTING_DOWN;
  Mode pending_mode = STANDING;
  bool has_pending = false;
  smov::TrigonometryState trig;  <!-- Nutzt TrigonometryState aus smov_lib -->
  smov::Vector3 coord1, coord2, coord3, coord4;
  bool leg1_motion_done = true, leg2_motion_done = true, leg3_motion_done = true, leg4_motion_done = true;
  bool has_finished_walk = false, done_once = false, request_to_stop_walk = false, has_finished_turn = false;
  struct termios old_chars, new_chars;

 private:
  rclcpp::Publisher<i2cpwm_board_msgs::msg::ServoArray>::SharedPtr servo_pub_;
};
