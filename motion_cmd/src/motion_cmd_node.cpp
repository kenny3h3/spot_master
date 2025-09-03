#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include "smov_msgs/msg/states_servos.hpp"

// SMOV IK-Header (liegen in smov_lib/include/smov/)
#include "smov/trigonometry.h"
#include "smov/mathematics.h"

#include "motion_cmd/teleop_keys.hpp"

using namespace std::chrono_literals;

namespace {
inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
}

class MotionCmdNode : public rclcpp::Node {
public:
  MotionCmdNode() : Node("motion_cmd") {
    // Parameter (bei Bedarf in Launch überschreibbar)
    update_rate_hz_   = declare_parameter<double>("update_rate_hz", 100.0);
    gait_freq_hz_     = declare_parameter<double>("gait_frequency_hz", 1.8);
    step_height_      = declare_parameter<double>("step_height", 2.0);
    neutral_y_        = declare_parameter<double>("neutral_y",  6.0);
    neutral_x_front_  = declare_parameter<double>("neutral_x_front", 3.5);
    neutral_x_back_   = declare_parameter<double>("neutral_x_back",  3.7);
    max_stride_x_     = declare_parameter<double>("max_stride_x",   2.0);
    max_stride_y_     = declare_parameter<double>("max_stride_y",   2.0);

    upper_leg_length_  = declare_parameter<double>("upper_leg_length", 10.75);
    lower_leg_length_  = declare_parameter<double>("lower_leg_length", 13.0);
    hip_body_distance_ = declare_parameter<double>("hip_body_distance", 5.5);

    // Publisher wie im SMOV-Stack
    front_pub_ = this->create_publisher<smov_msgs::msg::StatesServos>("front_proportional_servos", 10);
    back_pub_  = this->create_publisher<smov_msgs::msg::StatesServos>("back_proportional_servos", 10);
    front_servos_.state_name = "MOTION_CMD";
    back_servos_.state_name  = "MOTION_CMD";

    // IK-State aus SMOV
    trig_ = std::make_unique<smov::TrigonometryState>(
      &front_servos_, &back_servos_, &front_pub_, &back_pub_,
      &upper_leg_length_, &lower_leg_length_, &hip_body_distance_
    );

    // Tastatur
    if (!keys_.init()) {
      RCLCPP_WARN(get_logger(), "Terminal konnte nicht in raw-mode gesetzt werden; Teleop evtl. inaktiv.");
    }

    // Timer Loop
    const double dt = 1.0 / update_rate_hz_;
    timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&MotionCmdNode::loop, this));

    RCLCPP_INFO(get_logger(), "motion_cmd started (W=start, S=stop, arrows=stride/strafe).");
  }

  ~MotionCmdNode() override { keys_.shutdown(); }

private:
  // Steuerzustand
  bool walking_{false};
  double phase_{0.0};
  double vx_{0.0}, vy_{0.0};             // vor/zurück, strafe
  const double base_speed_x_ = 0.8;      // Tastendruck -> gewünschte Schrittweite
  const double base_speed_y_ = 0.8;

  // Parameter
  double update_rate_hz_, gait_freq_hz_, step_height_;
  double neutral_y_, neutral_x_front_, neutral_x_back_;
  double max_stride_x_, max_stride_y_;
  double upper_leg_length_, lower_leg_length_, hip_body_distance_;

  // SMOV
  rclcpp::Publisher<smov_msgs::msg::StatesServos>::SharedPtr front_pub_, back_pub_;
  smov_msgs::msg::StatesServos front_servos_, back_servos_;
  std::unique_ptr<smov::TrigonometryState> trig_;

  // infra
  rclcpp::TimerBase::SharedPtr timer_;
  motion_cmd::TeleopKeys keys_;

  // Hilfs-Fußbahn (Swing-Bogen)
  float swing_z(double ph) const {
    if (ph < 0.5) return 5.0f;
    double t = (ph - 0.5) / 0.5; // 0..1
    return 5.0f + float(step_height_) * float(std::sin(M_PI * t));
  }

  float step_x(double neutral_x, double ph, double stride) const {
    if (ph < 0.5) {
      double t = ph / 0.5;            // stance
      return float(neutral_x - stride * (1.0 - t));
    } else {
      double t = (ph - 0.5) / 0.5;    // swing
      return float(neutral_x + stride * t);
    }
  }

  void loop() {
    // Tastatur lesen
    if (auto k = keys_.readKey()) {
      const std::string &key = *k;
      if (key.size() == 1) {
        const char c = std::tolower(key[0]);
        if (c == 'w') { walking_ = true;  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "WALK ON"); }
        if (c == 's') { walking_ = false; vx_ = 0.0; vy_ = 0.0; RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "WALK OFF"); }
        if (c == ' ') { vx_ = 0.0; vy_ = 0.0; }
      } else {
        if (key == "UP")    vx_ =  base_speed_x_;
        if (key == "DOWN")  vx_ = -base_speed_x_;
        if (key == "LEFT")  vy_ = -base_speed_y_;
        if (key == "RIGHT") vy_ =  base_speed_y_;
      }
    }

    // Phase updaten
    const double dt = 1.0 / update_rate_hz_;
    if (walking_) {
      phase_ += dt * gait_freq_hz_;
      if (phase_ >= 1.0) phase_ -= 1.0;
    }

    // Gait berechnen (Trot: (FL,BR)=ph0; (FR,BL)=ph1)
    const double ph0 = phase_;
    const double ph1 = std::fmod(phase_ + 0.5, 1.0);
    const double stride_x = clampf(vx_, -max_stride_x_, max_stride_x_);
    const double stride_y = clampf(vy_, -max_stride_y_, max_stride_y_);

    smov::Vector3 fl, fr, bl, br;

    if (!walking_) {
      fl = {float(neutral_x_front_), float(neutral_y_), 5.0f};
      fr = {float(neutral_x_front_), float(neutral_y_), 5.0f};
      bl = {float(neutral_x_back_),  float(neutral_y_), 5.0f};
      br = {float(neutral_x_back_),  float(neutral_y_), 5.0f};
    } else {
      fl.x = step_x(neutral_x_front_, ph0, stride_x); fl.y = float(neutral_y_ + stride_y); fl.z = swing_z(ph0);
      br.x = step_x(neutral_x_back_,  ph0, stride_x); br.y = float(neutral_y_ + stride_y); br.z = swing_z(ph0);

      fr.x = step_x(neutral_x_front_, ph1, stride_x); fr.y = float(neutral_y_ + stride_y); fr.z = swing_z(ph1);
      bl.x = step_x(neutral_x_back_,  ph1, stride_x); bl.y = float(neutral_y_ + stride_y); bl.z = swing_z(ph1);
    }

    // IK anwenden → publisht StatesServos auf die beiden Topics
    // Leg-IDs angenommen: 1=FL, 2=FR, 3=BL, 4=BR
    trig_->set_leg_to(1, fl);
    trig_->set_leg_to(2, fr);
    trig_->set_leg_to(3, bl);
    trig_->set_leg_to(4, br);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionCmdNode>());
  rclcpp::shutdown();
  return 0;
}
