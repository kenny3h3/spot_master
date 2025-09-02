#include "walking_gait.h"

float ForwardMotion::curved(float x, float dist_from_origin, float gap) {
  return -sqrt(25.0f - pow((2 * x - dist_from_origin), 2)) + neutral_y + gap;
}

void ForwardMotion::stabilize_legs() {
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

  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Set default position to ((x≈%.2f/%.2f, y≈%.2f, z=5)", neutral_x_front, neutral_x_back, neutral_y);
}

void ForwardMotion::output_coordinates() {
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "REMINDER: (Press once) Up arrow key to move forward");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "                       Down arrow key to stop moving");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "                       Right arrow key to turn right");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "                       Left arrow key to turn left");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Coordinates leg 1: (%f, %f, %f)", coord1.x, coord1.y, coord1.z);
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Coordinates leg 2: (%f, %f, %f)", coord2.x, coord2.y, coord2.z);
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Coordinates leg 3: (%f, %f, %f)", coord3.x, coord3.y, coord3.z);
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Coordinates leg 4: (%f, %f, %f)", coord4.x, coord4.y, coord4.z);
  if (mode == SITTING_DOWN)
    RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot Mode:        SITTING_DOWN");
  else if (mode == WAKING_UP) 
    RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot Mode:        WAKING_UP");
  else if (mode == STANDING) 
    RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot Mode:        STANDING");
  else if (mode == WALKING) 
    RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot Mode:        WALKING");
  else if (mode == TURNING_RIGHT)
    RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot Mode:        TURNING_RIGHT");
  else 
    RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Robot Mode:        TURNING_LEFT");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Motion done leg 1: %d", leg1_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Motion done leg 2: %d", leg2_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Motion done leg 3: %d", leg3_motion_done);
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "Motion done leg 4: %d", leg4_motion_done);
}

void ForwardMotion::walk() {
  if (!leg1_motion_done) {
    if (coord1.x > -1.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, -1.5f, 0.15f);
      coord1.y = curved(coord1.x, 2.0f, 0.0f);
      trig.set_leg_to(1, coord1);
    } else {
      leg1_motion_done = true;
      if (!request_to_stop_walk) leg2_motion_done = false;
    }
  } else {
    if (coord1.x < 3.45f) {
      coord1.x = smov::Functions::lerp(coord1.x, 3.5f, 0.15f);
      trig.set_leg_to(1, coord1);
    }
  }

  if (!leg4_motion_done) {
    if (coord4.x > -1.45f) {
      coord4.x = smov::Functions::lerp(coord4.x, -1.5f, 0.15f);
      coord4.y = curved(coord4.x, 2.0f, back_leg_gap);
      trig.set_leg_to(4, coord4);
    } else {
      leg4_motion_done = true;
      if (!request_to_stop_walk) leg3_motion_done = false;
    }
  } else {
    if (coord4.x < 3.45f) {
      coord4.x = smov::Functions::lerp(coord4.x, 3.5f, 0.15f);
      trig.set_leg_to(4, coord4);
    }
  }

  if (!leg2_motion_done) {
    if (coord2.x > -1.45f) {
      coord2.x = smov::Functions::lerp(coord2.x, -1.5f, 0.15f);
      coord2.y = curved(coord2.x, 2.0f, 0.0f);
      trig.set_leg_to(2, coord2);
    } else {
      leg2_motion_done = true;
      if (!request_to_stop_walk) leg1_motion_done = false;
    }
  } else {
    if (coord2.x < 3.45f) {
      coord2.x = smov::Functions::lerp(coord2.x, 3.5f, 0.15f);
      trig.set_leg_to(2, coord2);
    }
  }

  if (!leg3_motion_done) {
    if (coord3.x > -1.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, -1.5f, 0.15f);
      coord3.y = curved(coord3.x, 2.0f, back_leg_gap);
      trig.set_leg_to(3, coord3);
    } else {
      leg3_motion_done = true;
      if (!request_to_stop_walk) leg4_motion_done = false;
    }
  } else {
    if (coord3.x < 3.45f) {
      coord3.x = smov::Functions::lerp(coord3.x, 3.5f, 0.15f);
      trig.set_leg_to(3, coord3);
    }
  }
}

void ForwardMotion::turn() {
  if (!leg1_motion_done) {
    if (coord1.z < 9.95f) {
      coord1.z = smov::Functions::lerp(coord1.z, 10.0f, 0.15f);
      coord1.y = curved(coord1.z, 10.0f, 0.0f);
      trig.set_leg_to(1, coord1);
    } else {
      leg1_motion_done = true;
      if (!request_to_stop_walk) leg2_motion_done = false;
    }
  } else {
    if (coord1.z > 4.95f) {
      coord1.z = smov::Functions::lerp(coord1.z, 5.0f, 0.15f);
      trig.set_leg_to(1, coord1);
    }
  }

  if (!leg4_motion_done) {
    if (coord4.z < 9.95f) {
      coord4.z = smov::Functions::lerp(coord4.z, 10.0f, 0.15f);
      coord4.y = curved(coord4.z, 10.0f, back_leg_gap);
      trig.set_leg_to(4, coord4);
    } else {
      leg4_motion_done = true;
      if (!request_to_stop_walk) leg3_motion_done = false;
    }
  } else {
    if (coord4.z > 4.95f) {
      coord4.z = smov::Functions::lerp(coord4.z, 5.0f, 0.15f);
      trig.set_leg_to(4, coord4);
    }
  }

  if (!leg2_motion_done) {
    if (coord2.z < 9.95f) {
      coord2.z = smov::Functions::lerp(coord2.z, 10.0f, 0.15f);
      coord2.y = curved(coord2.z, 10.0f, 0.0f);
      trig.set_leg_to(2, coord2);
    } else {
      leg2_motion_done = true;
      if (!request_to_stop_walk) leg1_motion_done = false;
    }
  } else {
    if (coord2.z > 4.95f) {
      coord2.z = smov::Functions::lerp(coord2.z, 5.0f, 0.15f);
      trig.set_leg_to(2, coord2);
    }
  }

  if (!leg3_motion_done) {
    if (coord3.z < 9.95f) {
      coord3.z = smov::Functions::lerp(coord3.z, 10.0f, 0.15f);
      coord3.y = curved(coord3.z, 10.0f, back_leg_gap);
      trig.set_leg_to(3, coord3);
    } else {
      leg3_motion_done = true;
      if (!request_to_stop_walk) leg4_motion_done = false;
    }
  } else {
    if (coord3.z > 4.95f) {
      coord3.z = smov::Functions::lerp(coord3.z, 5.0f, 0.15f);
      trig.set_leg_to(3, coord3);
    }
  }
}

void ForwardMotion::wake_up() {
  // We add a tiny cooldown for safety & to make sure
  // it does the command succesfully.
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "\033[2J\033[;H");
  RCLCPP_INFO(rclcpp::get_logger("walking_gait"), "[WAKING UP]");
  smov::delay(800);
  for (int i = 0; i < 2; i++) {
    front_servos.value[i] = 90.0f;
    front_servos.value[i + 2] = 55.0f;
    front_servos.value[i + 4] = 45.0f;
    back_servos.value[i] = 120.0f;
    back_servos.value[i + 2] = 150.0f;
    back_servos.value[i + 4] = 45.0f;
  }
  
  // We publish the values here as the main node is being locked.
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed first sequence to wake up.");

  // To mark a transition.
  smov::delay(800);

  // Doing the separate waking up phase to the back servos.
  for (int i = 0; i < 2; i++) 
    back_servos.value[i] = 90.0f;
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed second sequence to wake up.");

  // One more transition.
  smov::delay(800);

  // Executing the last sequence.
  for (int i = 0; i < 2; i++) {
    front_servos.value[i + 2] = 45.0f;
    front_servos.value[i + 4] = 112.0f;
  }
  for (int i = 0; i < 2; i++) {
    back_servos.value[i + 2] = 45.0f;
    back_servos.value[i + 4] = 115.0f;
  }
  
  // We publish the values here as the main node is being locked.
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot may have woken up!");

  smov::delay(2000);
  mode = STANDING;
  stabilize_legs();
}

void ForwardMotion::on_start() {
  // Getting the default config.
  tcgetattr(0, &old_chars);

  // Initializing the reader.
  fcntl(0, F_SETFL, O_NONBLOCK);
  new_chars = old_chars;
  new_chars.c_lflag &= ~ICANON;
  new_chars.c_lflag &= 0 ? ECHO : ~ECHO; // echo = 0.
  tcsetattr(0, TCSANOW, &new_chars);
}

void ForwardMotion::on_loop() {
  int c = getchar();
  switch (c) {
    case 65: // 65: Key up.
      if (mode == STANDING && mode != SITTING_DOWN) {
        mode = WALKING;
        request_to_stop_walk = false;
      }
      break;
    case 66: // 66: Key down. 
      if (mode != SITTING_DOWN) {
        if (has_finished_walk && has_finished_turn) 
          mode = STANDING;
        else 
          request_to_stop_walk = true;
        }
      break;
    case 67: // 67: Key right.
      if (mode != SITTING_DOWN) {
        request_to_stop_walk = false;
        if (mode == STANDING) mode = TURNING_RIGHT;
      }
      break;
    case 68:
      if (mode != SITTING_DOWN) {
        request_to_stop_walk = false;
        if (mode == STANDING) mode = TURNING_LEFT;
      }
      break;
    case ' ':
      if (mode == SITTING_DOWN) {
        mode = WAKING_UP;
        wake_up();
      }
  }

  output_coordinates();

  if (smov::Functions::approx(coord1.x, 3.5f, 0.06f) && smov::Functions::approx(coord2.x, 3.5f, 0.06f) 
    && smov::Functions::approx(coord3.x, 3.5f, 0.06f) && smov::Functions::approx(coord4.x, 3.5f, 0.06f) && request_to_stop_walk) {
    has_finished_walk = true;
    if (mode == STANDING) done_once = false;
  } else 
    has_finished_walk = false;

  if (mode == WALKING) {
    if (done_once == false) {
      // Some code that executes only once.
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
    }
    walk();
  }

  if (mode == TURNING_RIGHT) {
    if (done_once == false) {
      // Some code that executes only once.
      leg1_motion_done = false;
      leg4_motion_done = false;
      done_once = true;
    }
    turn();
  }

  if (mode == TURNING_LEFT) {
    if (done_once == false) {
      // Some code that executes only once.
      leg2_motion_done = false;
      leg3_motion_done = false;
      done_once = true;
    }
    turn();
  }

    if (smov::Functions::approx(coord1.z, 5.0f, 0.06f) && smov::Functions::approx(coord2.z, 5.0f, 0.06f) 
      && smov::Functions::approx(coord3.z, 5.0f, 0.06f) && smov::Functions::approx(coord4.z, 5.0f, 0.06f) && request_to_stop_walk) {
      has_finished_turn = true;
      if (mode == STANDING) done_once = false;
    } else 
      has_finished_turn = false;

    if (smov::Functions::approx(coord1.z, 5.0f, 0.06f) && smov::Functions::approx(coord2.z, 5.0f, 0.06f) && 
      smov::Functions::approx(coord3.z, 5.0f, 0.06f) && smov::Functions::approx(coord4.z, 5.0f, 0.06f) && 
      smov::Functions::approx(coord1.x, 3.5f, 0.06f) && smov::Functions::approx(coord2.x, 3.5f, 0.06f) && 
      smov::Functions::approx(coord3.x, 3.5f, 0.06f) && smov::Functions::approx(coord4.x, 3.5f, 0.06f) && 
      request_to_stop_walk) {
      mode = STANDING;
    }
}

void ForwardMotion::on_quit() {
  // Changing to default config.
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

DECLARE_STATE_NODE_CLASS("walking_gait", ForwardMotion, 15ms)
