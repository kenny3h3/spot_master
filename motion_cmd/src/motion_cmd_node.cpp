#include <chrono>
#include <cmath>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "smov_msgs/msg/states_servos.hpp"

// SMOV IK
#include "smov/trigonometry.h"
#include "smov/mathematics.h"

#include "motion_cmd/teleop_keys.hpp"

using namespace std::chrono_literals;

namespace { inline float clampf(float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); } inline double smoothstep(double t){ return t*t*(3-2*t); } }

class MotionCmdNode : public rclcpp::Node {
public:
  MotionCmdNode() : Node("motion_cmd") {
    // Tuning
    update_rate_hz_   = declare_parameter<double>("update_rate_hz", 100.0);
    gait_freq_hz_     = declare_parameter<double>("gait_frequency_hz", 1.8);
    step_height_      = declare_parameter<double>("step_height", 2.0);
    neutral_y_        = declare_parameter<double>("neutral_y",  6.0);
    neutral_x_front_  = declare_parameter<double>("neutral_x_front", 3.5);
    neutral_x_back_   = declare_parameter<double>("neutral_x_back",  3.7);
    max_stride_x_     = declare_parameter<double>("max_stride_x",   2.0);
    max_turn_         = declare_parameter<double>("max_turn",       1.0);

    // Wake-up Sequenz
    wake_duration_    = declare_parameter<double>("wake_duration", 2.5); // s
    wake_start_z_     = declare_parameter<double>("wake_start_z",  2.5);
    wake_end_z_       = declare_parameter<double>("wake_end_z",    5.0);

    // IK-Geometrie
    upper_leg_length_  = declare_parameter<double>("upper_leg_length", 10.75);
    lower_leg_length_  = declare_parameter<double>("lower_leg_length", 13.0);
    hip_body_distance_ = declare_parameter<double>("hip_body_distance", 5.5);

    // Publisher
    front_pub_ = this->create_publisher<smov_msgs::msg::StatesServos>("front_proportional_servos", 10);
    back_pub_  = this->create_publisher<smov_msgs::msg::StatesServos>("back_proportional_servos", 10);
    front_servos_.state_name = "MOTION_CMD";
    back_servos_.state_name  = "MOTION_CMD";

    // IK
    trig_ = std::make_unique<smov::TrigonometryState>(
      &front_servos_, &back_servos_, &front_pub_, &back_pub_,
      &upper_leg_length_, &lower_leg_length_, &hip_body_distance_
    );

    // Tastatur optional
    if (!keys_.init()) {
      RCLCPP_WARN(get_logger(), "Kein Terminal-Rawmode: Steuerung per /cmd_vel, /walk_cmd, /idle_cmd empfohlen.");
    } else {
      RCLCPP_INFO(get_logger(), "Tasten: W=Aufwachen+Gehen, S=Stop, Pfeile: vor/zurück & drehen.");
    }

    // Topic-Steuerung
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        vx_ = clampf(float(msg->linear.x), -1.0f, 1.0f) * base_speed_x_;
        wz_ = clampf(float(msg->angular.z), -1.0f, 1.0f) * base_turn_;
        if (mode_ == Mode::IDLE && (std::fabs(vx_) > 1e-3f || std::fabs(wz_) > 1e-3f)) startWake();
      });

    walk_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/walk_cmd", 10, [this](const std_msgs::msg::Bool::SharedPtr m){ if(m->data) startWake(); });
    idle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/idle_cmd", 10, [this](const std_msgs::msg::Bool::SharedPtr m){ if(m->data) goIdle(); });

    // Timer
    const double dt = 1.0 / update_rate_hz_;
    timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&MotionCmdNode::loop, this));

    RCLCPP_INFO(get_logger(), "motion_cmd: IDLE bereit.");
  }

  ~MotionCmdNode() override { keys_.shutdown(); }

private:
  enum class Mode { IDLE, WAKE, WALK };
  Mode mode_{Mode::IDLE};

  // Steuerzustand
  double phase_{0.0};
  float vx_{0.0f}; // vor/zurück
  float wz_{0.0f}; // drehen
  const float base_speed_x_ = 0.8f;
  const float base_turn_    = 0.6f;

  // Parameter
  double update_rate_hz_, gait_freq_hz_, step_height_;
  double neutral_y_, neutral_x_front_, neutral_x_back_;
  double max_stride_x_, max_turn_;
  double upper_leg_length_, lower_leg_length_, hip_body_distance_;
  double wake_duration_, wake_start_z_, wake_end_z_;

  // SMOV
  rclcpp::Publisher<smov_msgs::msg::StatesServos>::SharedPtr front_pub_, back_pub_;
  smov_msgs::msg::StatesServos front_servos_, back_servos_;
  std::unique_ptr<smov::TrigonometryState> trig_;

  // infra
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr walk_sub_, idle_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  motion_cmd::TeleopKeys keys_;

  // Wake intern
  double wake_t_{0.0};

  // Hilfsfunktionen
  float swing_z(double ph) const {
    if (ph < 0.5) return float(wake_end_z_);          // stance auf Standhöhe
    double t = (ph - 0.5) / 0.5;
    return float(wake_end_z_) + float(step_height_) * float(std::sin(M_PI * t));
  }

  float step_x(double neutral_x, double ph, double stride) const {
    if (ph < 0.5) { double t = ph / 0.5; return float(neutral_x - stride * (1.0 - t)); }
    else          { double t = (ph - 0.5) / 0.5; return float(neutral_x + stride * t); }
  }

  void startWake() {
    if (mode_ == Mode::IDLE) {
      mode_ = Mode::WAKE; wake_t_ = 0.0;
      RCLCPP_INFO(get_logger(), "Wake-Up gestartet …");
    } else if (mode_ == Mode::WALK) {
      // schon wach: einfach weiter
    } else {
      // WAKE läuft schon
    }
  }
  void goIdle() {
    mode_ = Mode::IDLE; vx_ = 0.0f; wz_ = 0.0f; phase_ = 0.0;
    // neutrale Pose
    smov::Vector3 fl{float(neutral_x_front_), float(neutral_y_), float(wake_end_z_)};
    smov::Vector3 fr{float(neutral_x_front_), float(neutral_y_), float(wake_end_z_)};
    smov::Vector3 bl{float(neutral_x_back_),  float(neutral_y_), float(wake_end_z_)};
    smov::Vector3 br{float(neutral_x_back_),  float(neutral_y_), float(wake_end_z_)};
    trig_->set_leg_to(1, fl); trig_->set_leg_to(2, fr); trig_->set_leg_to(3, bl); trig_->set_leg_to(4, br);
    RCLCPP_INFO(get_logger(), "IDLE.");
  }

  void handle_keyboard() {
    auto kk = keys_.readKey(); if (!kk) return;
    const std::string &k = *kk;
    if (k.size()==1) {
      const char c = std::tolower(k[0]);
      if (c=='w') startWake();
      if (c=='s') goIdle();
      if (c==' ') { vx_=0.0f; wz_=0.0f; }
    } else {
      if (k=="UP")    vx_ =  base_speed_x_;
      if (k=="DOWN")  vx_ = -base_speed_x_;
      if (k=="LEFT")  wz_ =  base_turn_;   // links drehen
      if (k=="RIGHT") wz_ = -base_turn_;   // rechts drehen
    }
  }

  void loop() {
    handle_keyboard();

    const double dt = 1.0 / update_rate_hz_;

    if (mode_ == Mode::WAKE) {
      // sanft von (x_neutral, y_neutral, wake_start_z) nach (… , wake_end_z)
      wake_t_ += dt;
      double u = std::min(1.0, wake_t_ / wake_duration_);
      double s = smoothstep(u);

      const float z = float(wake_start_z_ + (wake_end_z_ - wake_start_z_) * s);

      smov::Vector3 fl{float(neutral_x_front_), float(neutral_y_), z};
      smov::Vector3 fr{float(neutral_x_front_), float(neutral_y_), z};
      smov::Vector3 bl{float(neutral_x_back_),  float(neutral_y_), z};
      smov::Vector3 br{float(neutral_x_back_),  float(neutral_y_), z};
      trig_->set_leg_to(1, fl); trig_->set_leg_to(2, fr); trig_->set_leg_to(3, bl); trig_->set_leg_to(4, br);

      if (u >= 1.0) {
        mode_ = Mode::WALK;
        RCLCPP_INFO(get_logger(), "Wake-Up fertig → WALK.");
      }
      return; // in WAKE keine Schrittphase
    }

    if (mode_ == Mode::IDLE) {
      // stehende Pose halten
      smov::Vector3 fl{float(neutral_x_front_), float(neutral_y_), float(wake_end_z_)};
      smov::Vector3 fr{float(neutral_x_front_), float(neutral_y_), float(wake_end_z_)};
      smov::Vector3 bl{float(neutral_x_back_),  float(neutral_y_), float(wake_end_z_)};
      smov::Vector3 br{float(neutral_x_back_),  float(neutral_y_), float(wake_end_z_)};
      trig_->set_leg_to(1, fl); trig_->set_leg_to(2, fr); trig_->set_leg_to(3, bl); trig_->set_leg_to(4, br);
      return;
    }

    // WALK
    phase_ += dt * gait_freq_hz_;
    if (phase_ >= 1.0) phase_ -= 1.0;

    const double ph0 = phase_;
    const double ph1 = std::fmod(phase_ + 0.5, 1.0);
    const double stride_x = clampf(vx_, -float(max_stride_x_), float(max_stride_x_));
    const double turn     = clampf(wz_, -float(max_turn_),     float(max_turn_));

    smov::Vector3 fl, fr, bl, br;
    // Dreh-Differential in X (links+, rechts-)
    const double stride_fl = stride_x + turn;
    const double stride_fr = stride_x - turn;
    const double stride_bl = stride_x + turn;
    const double stride_br = stride_x - turn;

    fl.x = step_x(neutral_x_front_, ph0, stride_fl); fl.y = float(neutral_y_); fl.z = swing_z(ph0);
    br.x = step_x(neutral_x_back_,  ph0, stride_br); br.y = float(neutral_y_); br.z = swing_z(ph0);

    fr.x = step_x(neutral_x_front_, ph1, stride_fr); fr.y = float(neutral_y_); fr.z = swing_z(ph1);
    bl.x = step_x(neutral_x_back_,  ph1, stride_bl); bl.y = float(neutral_y_); bl.z = swing_z(ph1);

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
