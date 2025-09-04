#include "spot_micro_motion_cmd/spot_micro_motion_cmd.h"
#include "smov/configure_servos.h"  // Für Servo-Konfiguration
#include "utils.h"

SpotMicroMotionCmd::SpotMicroMotionCmd() : trig(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, &upper_leg_length, &lower_leg_length, &hip_body_distance) {
  servo_pub_ = this->create_publisher<i2cpwm_board_msgs::msg::ServoArray>("/servos_absolute", 10);

  // Lade Konfig aus smov_config
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "smov_config");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for smov_config service.");
      return;
    }
  }
  auto params = param_client->get_parameters({
    "use_single_board", "front_board_i2c_bus", "back_board_i2c_bus",
    "FRONT_BODY_LEFT", "FRONT_BODY_RIGHT", "FRONT_UPPER_LEG_LEFT",
    "FRONT_UPPER_LEG_RIGHT", "FRONT_LOWER_LEG_LEFT", "FRONT_LOWER_LEG_RIGHT",
    "BACK_BODY_LEFT", "BACK_BODY_RIGHT", "BACK_UPPER_LEG_LEFT",
    "BACK_UPPER_LEG_RIGHT", "BACK_LOWER_LEG_LEFT", "BACK_LOWER_LEG_RIGHT"
  });
  // Speichere Parameter (vereinfacht, Array-Handling erweitern)

  // Lade State-Parameter aus smov_states
  auto state_client = std::make_shared<rclcpp::SyncParametersClient>(this, "smov_states");
  auto state_params = state_client->get_parameters({"upper_leg_length", "lower_leg_length", "hip_body_distance"});
  upper_leg_length = state_params[0].as_double();
  lower_leg_length = state_params[1].as_double();
  hip_body_distance = state_params[2].as_double();
}

SpotMicroMotionCmd::~SpotMicroMotionCmd() {}

float SpotMicroMotionCmd::curved(float x, float dist_from_origin, float gap) {
  return -sqrt(25.0f - pow((2 * x - dist_from_origin), 2)) + 15.5f + gap; // Höhe anpassen
}

void SpotMicroMotionCmd::stabilize_legs() {
  coord1 = smov::Vector3(3.5f, 15.5f, 5.0f);
  coord2 = smov::Vector3(3.5f, 15.5f, 5.0f);
  coord3 = smov::Vector3(3.7f, 15.5f, 5.0f); // Back legs offset
  coord4 = smov::Vector3(3.7f, 15.5f, 5.0f);
  trig.set_leg_to(1, coord1);
  trig.set_leg_to(2, coord2);
  trig.set_leg_to(3, coord3);
  trig.set_leg_to(4, coord4);
  RCLCPP_INFO(this->get_logger(), "Stabilized legs at default position.");
}

void SpotMicroMotionCmd::output_coordinates() {
  RCLCPP_INFO(this->get_logger(), "\033[2J\033[;H");
  RCLCPP_INFO(this->get_logger(), "REMINDER: (Press once) W key to wake up");
  RCLCPP_INFO(this->get_logger(), "                       Up arrow key to move forward");
  RCLCPP_INFO(this->get_logger(), "                       Up + Left/Right arrow to curve left/right");
  RCLCPP_INFO(this->get_logger(), "                       S key to stop moving");
  RCLCPP_INFO(this->get_logger(), "                       Right arrow key to turn right");
  RCLCPP_INFO(this->get_logger(), "                       Left arrow key to turn left");
  RCLCPP_INFO(this->get_logger(), "Mode: %d", mode);
  RCLCPP_INFO(this->get_logger(), "Leg 1: (%.2f, %.2f, %.2f)", coord1.x, coord1.y, coord1.z);
}

void SpotMicroMotionCmd::walk() {
  RCLCPP_INFO(this->get_logger(), "Executing walk()");
  if (!leg1_motion_done) {
    coord1.x = smov::Functions::lerp(coord1.x, -1.5f, 0.15f);
    coord1.y = curved(coord1.x, 2.0f, 0.0f);
    trig.set_leg_to(1, coord1);
    leg1_motion_done = true;
  }
}

void SpotMicroMotionCmd::turn() {
  RCLCPP_INFO(this->get_logger(), "Executing turn()");
  if (!leg1_motion_done) {
    coord1.z = smov::Functions::lerp(coord1.z, 10.0f, 0.15f);
    trig.set_leg_to(1, coord1);
    leg1_motion_done = true;
  }
}

void SpotMicroMotionCmd::curve(bool is_left) {
  RCLCPP_INFO(this->get_logger(), "Executing curve(%d)", is_left);
  if (!leg1_motion_done) {
    coord1.y += is_left ? 0.01f : -0.01f;
    trig.set_leg_to(1, coord1);
    leg1_motion_done = true;
  }
}

void SpotMicroMotionCmd::wake_up() {
  RCLCPP_INFO(this->get_logger(), "[WAKING UP]");
  rclcpp::sleep_for(800ms);
  stabilize_legs();
  mode = STANDING;
}

void SpotMicroMotionCmd::set_servo_positions(const smov::Vector3& coords) {
  i2cpwm_board_msgs::msg::ServoArray msg;
  msg.servos.resize(12);
  // Nutze trig.set_leg_to Ergebnisse, um Servo-Winkel zu berechnen (vereinfacht)
  // Hier müsste die Zuordnung zu den 12 Servos aus smov_config erfolgen
  servo_pub_->publish(msg);
}

void SpotMicroMotionCmd::on_start() {
  tcgetattr(STDIN_FILENO, &old_chars);
  new_chars = old_chars;
  new_chars.c_lflag &= ~(ICANON | ECHO);
  new_chars.c_cc[VMIN] = 0;
  new_chars.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_chars);
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  RCLCPP_INFO(this->get_logger(), "Terminal initialized at %s", __TIME__);
}

void SpotMicroMotionCmd::on_loop() {
  char buf[3];
  int n;
  bool up_pressed = false, left_pressed = false, right_pressed = false;

  while ((n = read(STDIN_FILENO, buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; i++) {
      if (buf[i] == 27 && i + 2 < n && buf[i + 1] == '[') {
        i += 2;
        switch (buf[i]) {
          case 'A': up_pressed = true; RCLCPP_INFO(this->get_logger(), "Up arrow detected"); break;
          case 'C': right_pressed = true; RCLCPP_INFO(this->get_logger(), "Right arrow detected"); break;
          case 'D': left_pressed = true; RCLCPP_INFO(this->get_logger(), "Left arrow detected"); break;
        }
      } else if (buf[i] == 'w' || buf[i] == 'W') {
        if (mode == SITTING_DOWN) { mode = WAKING_UP; wake_up(); }
      } else if (buf[i] == 's' || buf[i] == 'S') {
        if (mode != SITTING_DOWN) request_to_stop_walk = true;
      }
    }
  }

  Mode desired_mode = STANDING;
  if (mode != SITTING_DOWN && mode != WAKING_UP) {
    if (up_pressed && left_pressed && !right_pressed) desired_mode = CURVING_LEFT;
    else if (up_pressed && right_pressed && !left_pressed) desired_mode = CURVING_RIGHT;
    else if (up_pressed && !left_pressed && !right_pressed) desired_mode = WALKING;
    else if (left_pressed && !up_pressed && !right_pressed) desired_mode = TURNING_LEFT;
    else if (right_pressed && !up_pressed && !left_pressed) desired_mode = TURNING_RIGHT;
  }

  if (desired_mode != STANDING && mode == STANDING) {
    mode = desired_mode;
    request_to_stop_walk = false;
    done_once = false;
    RCLCPP_INFO(this->get_logger(), "Transition to mode %d", mode);
  } else if (desired_mode != mode) {
    pending_mode = desired_mode;
    has_pending = true;
    request_to_stop_walk = true;
  }

  output_coordinates();

  has_finished_walk = smov::Functions::approx(coord1.x, 3.5f, 0.06f) &&
                      smov::Functions::approx(coord2.x, 3.5f, 0.06f) &&
                      smov::Functions::approx(coord3.x, 3.7f, 0.06f) &&
                      smov::Functions::approx(coord4.x, 3.7f, 0.06f) && request_to_stop_walk;
  has_finished_turn = true;

  if (mode == WALKING) {
    if (!done_once) { leg1_motion_done = false; done_once = true; }
    walk();
  } else if (mode == TURNING_LEFT || mode == TURNING_RIGHT) {
    if (!done_once) { leg1_motion_done = false; done_once = true; }
    turn();
  } else if (mode == CURVING_LEFT || mode == CURVING_RIGHT) {
    if (!done_once) { leg1_motion_done = false; done_once = true; }
    curve(mode == CURVING_LEFT);
  }

  if (has_finished_walk && has_finished_turn && request_to_stop_walk) {
    mode = STANDING;
    done_once = false;
    if (has_pending) {
      mode = pending_mode;
      has_pending = false;
      done_once = false;
    }
  }

  // Veröffentliche TF-Transformation für Debugging
  geometry_msgs::msg::Vector3 translation = {0.0, 0.0, 15.5f}; // Beispiel: Körperhöhe aus YAML
  geometry_msgs::msg::Vector3 euler_angles = {0.0, 0.0, 0.0};
  Utils::publishDynamicTransform(this, "base_link", "right_front_leg", translation, euler_angles);
}

void SpotMicroMotionCmd::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
  RCLCPP_INFO(this->get_logger(), "Restored terminal settings at %s", __TIME__);
}

DECLARE_STATE_NODE_CLASS("spot_micro_motion_cmd", SpotMicroMotionCmd, 15ms)
