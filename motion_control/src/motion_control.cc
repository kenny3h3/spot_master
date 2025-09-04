#include "motion_control/motion_control.h"

float MotionControl::curved(float x, float dist_from_origin, float gap) {
  return -sqrt(25.0f - pow((2 * x - dist_from_origin), 2)) + neutral_y + gap;
}

void MotionControl::stabilize_legs() {
  coord1.x = neutral_x_front;
  coord1.y = neutral_y;
  coord1.z = 5;

  coord2.x = neutral_x_front;
  coord2.y = neutral_y;
  coord2.z = 5;

  coord3.x = neutral_x_back;
  coord3.y = neutral_y + back_leg_gap;
  coord3.z = 5;

  coord4.x = neutral_x_back;
  coord4.y = neutral_y + back_leg_gap;
  coord4.z = 5;

  trig.set_leg_to(1, coord1);
  trig.set_leg_to(2, coord2);
  trig.set_leg_to(3, coord3);
  trig.set_leg_to(4, coord4);

  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Set default position to (x≈%.2f/%.2f, y≈%.2f, z=5)", neutral_x_front, neutral_x_back, neutral_y);
}

void MotionControl::output_coordinates() {
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "REMINDER: (Press once) W key to wake up");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "                       Up arrow key to move forward");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "                       Up + Left/Right arrow to curve left/right");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "                       S key to stop moving");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "                       Right arrow key to turn right");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "                       Left arrow key to turn left");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Coordinates leg 1: (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Coordinates leg 2: (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Coordinates leg 3: (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Coordinates leg 4: (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
  if (mode == SITTING_DOWN)
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        SITTING_DOWN");
  else if (mode == WAKING_UP) 
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        WAKING_UP");
  else if (mode == STANDING) 
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        STANDING");
  else if (mode == WALKING) 
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        WALKING");
  else if (mode == TURNING_RIGHT)
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        TURNING_RIGHT");
  else if (mode == TURNING_LEFT)
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        TURNING_LEFT");
  else if (mode == CURVING_LEFT)
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        CURVING_LEFT");
  else
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Robot Mode:        CURVING_RIGHT");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Motion done leg 1: %d", leg1_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Motion done leg 2: %d", leg2_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Motion done leg 3: %d", leg3_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Motion done leg 4: %d", leg4_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Pending mode: %d, Has pending: %d", pending_mode, has_pending);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "has_finished_walk: %d, has_finished_turn: %d, request_to_stop_walk: %d", has_finished_walk, has_finished_turn, request_to_stop_walk);
}

void MotionControl::walk() {
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Executing walk()");
  if (!leg1_motion_done) {
    if (coord1.x > -1.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, -1.5f, 0.15f);
      coord1.y = curved(coord1.x, 2.0f, 0.0f);
      trig.set_leg_to(1, coord1);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 1 moving to (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
    } else {
      leg1_motion_done = true;
      if (!request_to_stop_walk) leg2_motion_done = false;
    }
  } else {
    if (coord1.x < neutral_x_front - 0.05f) {
      coord1.x = smov::Functions::lerp(coord1.x, neutral_x_front, 0.15f);
      trig.set_leg_to(1, coord1);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 1 returning to (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
    }
  }

  if (!leg4_motion_done) {
    if (coord4.x > -1.45f) {
      coord4.x = smov::Functions::lerp(coord4.x, -1.5f, 0.15f);
      coord4.y = curved(coord4.x, 2.0f, back_leg_gap);
      trig.set_leg_to(4, coord4);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 4 moving to (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
    } else {
      leg4_motion_done = true;
      if (!request_to_stop_walk) leg3_motion_done = false;
    }
  } else {
    if (coord4.x < neutral_x_back - 0.05f) {
      coord4.x = smov::Functions::lerp(coord4.x, neutral_x_back, 0.15f);
      trig.set_leg_to(4, coord4);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 4 returning to (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
    }
  }

  if (!leg2_motion_done) {
    if (coord2.x > -1.45f) {
      coord2.x = smov::Functions::lerp(coord2.x, -1.5f, 0.15f);
      coord2.y = curved(coord2.x, 2.0f, 0.0f);
      trig.set_leg_to(2, coord2);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 2 moving to (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
    } else {
      leg2_motion_done = true;
      if (!request_to_stop_walk) leg1_motion_done = false;
    }
  } else {
    if (coord2.x < neutral_x_front - 0.05f) {
      coord2.x = smov::Functions::lerp(coord2.x, neutral_x_front, 0.15f);
      trig.set_leg_to(2, coord2);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 2 returning to (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
    }
  }

  if (!leg3_motion_done) {
    if (coord3.x > -1.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, -1.5f, 0.15f);
      coord3.y = curved(coord3.x, 2.0f, back_leg_gap);
      trig.set_leg_to(3, coord3);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 3 moving to (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
    } else {
      leg3_motion_done = true;
      if (!request_to_stop_walk) leg4_motion_done = false;
    }
  } else {
    if (coord3.x < neutral_x_back - 0.05f) {
      coord3.x = smov::Functions::lerp(coord3.x, neutral_x_back, 0.15f);
      trig.set_leg_to(3, coord3);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 3 returning to (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
    }
  }
}

void MotionControl::curve(bool is_left) {
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Executing curve(%d)", is_left);
  float z_target = is_left ? 7.0f : 3.0f;
  float z_return = 5.0f;

  if (!leg1_motion_done) {
    if (coord1.x > -1.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, -1.5f, 0.15f);
      coord1.y = curved(coord1.x, 2.0f, 0.0f);
      coord1.z = smov::Functions::lerp(coord1.z, z_target, 0.1f);
      trig.set_leg_to(1, coord1);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 1 moving to (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
    } else {
      leg1_motion_done = true;
      if (!request_to_stop_walk) leg2_motion_done = false;
    }
  } else {
    if (coord1.x < neutral_x_front - 0.05f) {
      coord1.x = smov::Functions::lerp(coord1.x, neutral_x_front, 0.15f);
      coord1.z = smov::Functions::lerp(coord1.z, z_return, 0.1f);
      trig.set_leg_to(1, coord1);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 1 returning to (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
    }
  }

  if (!leg4_motion_done) {
    if (coord4.x > -1.45f) {
      coord4.x = smov::Functions::lerp(coord4.x, -1.5f, 0.15f);
      coord4.y = curved(coord4.x, 2.0f, back_leg_gap);
      coord4.z = smov::Functions::lerp(coord4.z, z_target, 0.1f);
      trig.set_leg_to(4, coord4);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 4 moving to (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
    } else {
      leg4_motion_done = true;
      if (!request_to_stop_walk) leg3_motion_done = false;
    }
  } else {
    if (coord4.x < neutral_x_back - 0.05f) {
      coord4.x = smov::Functions::lerp(coord4.x, neutral_x_back, 0.15f);
      coord4.z = smov::Functions::lerp(coord4.z, z_return, 0.1f);
      trig.set_leg_to(4, coord4);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 4 returning to (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
    }
  }

  if (!leg2_motion_done) {
    if (coord2.x > -1.45f) {
      coord2.x = smov::Functions::lerp(coord2.x, -1.5f, 0.15f);
      coord2.y = curved(coord2.x, 2.0f, 0.0f);
      coord2.z = smov::Functions::lerp(coord2.z, z_target, 0.1f);
      trig.set_leg_to(2, coord2);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 2 moving to (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
    } else {
      leg2_motion_done = true;
      if (!request_to_stop_walk) leg1_motion_done = false;
    }
  } else {
    if (coord2.x < neutral_x_front - 0.05f) {
      coord2.x = smov::Functions::lerp(coord2.x, neutral_x_front, 0.15f);
      coord2.z = smov::Functions::lerp(coord2.z, z_return, 0.1f);
      trig.set_leg_to(2, coord2);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 2 returning to (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
    }
  }

  if (!leg3_motion_done) {
    if (coord3.x > -1.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, -1.5f, 0.15f);
      coord3.y = curved(coord3.x, 2.0f, back_leg_gap);
      coord3.z = smov::Functions::lerp(coord3.z, z_target, 0.1f);
      trig.set_leg_to(3, coord3);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 3 moving to (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
    } else {
      leg3_motion_done = true;
      if (!request_to_stop_walk) leg4_motion_done = false;
    }
  } else {
    if (coord3.x < neutral_x_back - 0.05f) {
      coord3.x = smov::Functions::lerp(coord3.x, neutral_x_back, 0.15f);
      coord3.z = smov::Functions::lerp(coord3.z, z_return, 0.1f);
      trig.set_leg_to(3, coord3);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 3 returning to (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
    }
  }
}

void MotionControl::turn() {
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Executing turn()");
  if (!leg1_motion_done) {
    if (coord1.z < 9.95f) {
      coord1.z = smov::Functions::lerp(coord1.z, 10.0f, 0.15f);
      coord1.y = curved(coord1.z, 15.0f, 0.0f);
      trig.set_leg_to(1, coord1);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 1 moving to (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
    } else {
      leg1_motion_done = true;
      if (!request_to_stop_walk) leg2_motion_done = false;
    }
  } else {
    if (coord1.z > 4.95f) {
      coord1.z = smov::Functions::lerp(coord1.z, 5.0f, 0.15f);
      trig.set_leg_to(1, coord1);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 1 returning to (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
    }
  }

  if (!leg4_motion_done) {
    if (coord4.z < 9.95f) {
      coord4.z = smov::Functions::lerp(coord4.z, 10.0f, 0.15f);
      coord4.y = curved(coord4.z, 15.0f, back_leg_gap);
      trig.set_leg_to(4, coord4);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 4 moving to (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
    } else {
      leg4_motion_done = true;
      if (!request_to_stop_walk) leg3_motion_done = false;
    }
  } else {
    if (coord4.z > 4.95f) {
      coord4.z = smov::Functions::lerp(coord4.z, 5.0f, 0.15f);
      trig.set_leg_to(4, coord4);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 4 returning to (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
    }
  }

  if (!leg2_motion_done) {
    if (coord2.z < 9.95f) {
      coord2.z = smov::Functions::lerp(coord2.z, 10.0f, 0.15f);
      coord2.y = curved(coord2.z, 15.0f, 0.0f);
      trig.set_leg_to(2, coord2);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 2 moving to (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
    } else {
      leg2_motion_done = true;
      if (!request_to_stop_walk) leg1_motion_done = false;
    }
  } else {
    if (coord2.z > 4.95f) {
      coord2.z = smov::Functions::lerp(coord2.z, 5.0f, 0.15f);
      trig.set_leg_to(2, coord2);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 2 returning to (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
    }
  }

  if (!leg3_motion_done) {
    if (coord3.z < 9.95f) {
      coord3.z = smov::Functions::lerp(coord3.z, 10.0f, 0.15f);
      coord3.y = curved(coord3.z, 15.0f, back_leg_gap);
      trig.set_leg_to(3, coord3);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 3 moving to (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
    } else {
      leg3_motion_done = true;
      if (!request_to_stop_walk) leg4_motion_done = false;
    }
  } else {
    if (coord3.z > 4.95f) {
      coord3.z = smov::Functions::lerp(coord3.z, 5.0f, 0.15f);
      trig.set_leg_to(3, coord3);
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Leg 3 returning to (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
    }
  }
}

void MotionControl::wake_up() {
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "[WAKING UP]");
  smov::delay(800);
  for (int i = 0; i < 2; i++) {
    front_servos.value[i] = 90.0f;
    front_servos.value[i + 2] = 55.0f;
    front_servos.value[i + 4] = 45.0f;
    back_servos.value[i] = 120.0f;
    back_servos.value[i + 2] = 150.0f;
    back_servos.value[i + 4] = 45.0f;
  }
  
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed first sequence to wake up.");
  smov::delay(800);

  for (int i = 0; i < 2; i++) 
    back_servos.value[i] = 90.0f;
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed second sequence to wake up.");
  smov::delay(800);

  for (int i = 0; i < 2; i++) {
    front_servos.value[i + 2] = 45.0f;
    front_servos.value[i + 4] = 112.0f;
    back_servos.value[i + 2] = 45.0f;
    back_servos.value[i + 4] = 115.0f;
  }
  
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot may have woken up!");
  smov::delay(2000);
  mode = STANDING;
  stabilize_legs();
  request_to_stop_walk = false;  // Ensure no stop request after wake up
}

void MotionControl::on_start() {
  tcgetattr(STDIN_FILENO, &old_chars);
  new_chars = old_chars;
  new_chars.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
  new_chars.c_cc[VMIN] = 0;  // Non-blocking read, return immediately
  new_chars.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_chars);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Terminal initialized for non-blocking input.");
}

void MotionControl::on_loop() {
  bool up_pressed = false, left_pressed = false, right_pressed = false;
  char buf[3];
  int n;

  // Read all available input
  while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; i++) {
      if (buf[i] == 27 && i + 2 < n && buf[i + 1] == '[') { // Escape sequence for arrow keys
        i += 2;
        switch (buf[i]) {
          case 'A': // Up arrow
            up_pressed = true;
            RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Up arrow detected");
            break;
          case 'C': // Right arrow
            right_pressed = true;
            RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Right arrow detected");
            break;
          case 'D': // Left arrow
            left_pressed = true;
            RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Left arrow detected");
            break;
        }
      } else if (buf[i] == 'w' || buf[i] == 'W') {
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "W key detected");
        if (mode == SITTING_DOWN) {
          mode = WAKING_UP;
          wake_up();
        }
      } else if (buf[i] == 's' || buf[i] == 'S') {
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "S key detected");
        if (mode != SITTING_DOWN) {
          request_to_stop_walk = true;
          has_pending = false;
        }
      }
    }
  }

  // Determine desired mode based on input
  Mode desired_mode = STANDING;
  if (mode != SITTING_DOWN && mode != WAKING_UP) {
    if (up_pressed && left_pressed && !right_pressed) {
      desired_mode = CURVING_LEFT;
    } else if (up_pressed && right_pressed && !left_pressed) {
      desired_mode = CURVING_RIGHT;
    } else if (up_pressed && !left_pressed && !right_pressed) {
      desired_mode = WALKING;
    } else if (left_pressed && !up_pressed && !right_pressed) {
      desired_mode = TURNING_LEFT;
    } else if (right_pressed && !up_pressed && !left_pressed) {
      desired_mode = TURNING_RIGHT;
    }
  }

  // Handle mode change
  if (desired_mode != STANDING) {
    if (mode == STANDING) {
      // Immediate transition from STANDING
      mode = desired_mode;
      request_to_stop_walk = false;
      done_once = false;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Direct transition from STANDING to mode %d", mode);
    } else if (desired_mode != mode) {
      // Queue for ongoing modes
      pending_mode = desired_mode;
      has_pending = true;
      request_to_stop_walk = true;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Queueing new mode: %d", desired_mode);
    }
  }

  output_coordinates();

  // Check if current motion is complete
  has_finished_walk = smov::Functions::approx(coord1.x, neutral_x_front, 0.06f) &&
                      smov::Functions::approx(coord2.x, neutral_x_front, 0.06f) &&
                      smov::Functions::approx(coord3.x, neutral_x_back, 0.06f) &&
                      smov::Functions::approx(coord4.x, neutral_x_back, 0.06f) &&
                      request_to_stop_walk;

  has_finished_turn = smov::Functions::approx(coord1.z, 5.0f, 0.06f) &&
                      smov::Functions::approx(coord2.z, 5.0f, 0.06f) &&
                      smov::Functions::approx(coord3.z, 5.0f, 0.06f) &&
                      smov::Functions::approx(coord4.z, 5.0f, 0.06f) &&
                      request_to_stop_walk;

  // Execute current mode
  if (mode == WALKING) {
    if (!done_once) {
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Starting WALKING cycle");
    }
    walk();
  } else if (mode == CURVING_LEFT) {
    if (!done_once) {
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Starting CURVING_LEFT cycle");
    }
    curve(true);
  } else if (mode == CURVING_RIGHT) {
    if (!done_once) {
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Starting CURVING_RIGHT cycle");
    }
    curve(false);
  } else if (mode == TURNING_RIGHT) {
    if (!done_once) {
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Starting TURNING_RIGHT cycle");
    }
    turn();
  } else if (mode == TURNING_LEFT) {
    if (!done_once) {
      leg2_motion_done = false;
      leg3_motion_done = false;
      done_once = true;
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Starting TURNING_LEFT cycle");
    }
    turn();
  }

  // Transition to pending mode when current motion is complete
  if (has_finished_walk && has_finished_turn && request_to_stop_walk) {
    mode = STANDING;
    done_once = false;
    request_to_stop_walk = false;
    if (has_pending) {
      RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Switching to pending mode: %d", pending_mode);
      mode = pending_mode;
      has_pending = false;
      done_once = false;
    }
  }
}

void MotionControl::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
  RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Restored terminal settings.");
}

DECLARE_STATE_NODE_CLASS("motion_control", MotionControl, 15ms)