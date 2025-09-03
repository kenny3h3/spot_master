#include <chrono>
#include <cmath>
#include <string>
#include <array>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "smov_msgs/msg/states_servos.hpp"
#include "smov/trigonometry.h"
#include "smov/mathematics.h"

#include "motion_cmd/teleop_keys.hpp"

using namespace std::chrono_literals;

namespace {

inline float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

} // namespace

class MotionCmdNode : public rclcpp::Node {
public:
  MotionCmdNode() : Node("motion_cmd") {
    // Parameter
    update_rate_hz_   = declare_parameter<double>("update_rate_hz", 100.0);
    gait_freq_hz_     = declare_parameter<double>("gait_frequency_hz", 1.8);
    step_height_      = declare_parameter<double>("step_height", 2.0);   // (relative z-Einheit)
    neutral_y_        = declare_parameter<double>("neutral_y",  6.0);    // seitliche Basis (in deinen smov-Koords)
    neutral_x_front_  = declare_parameter<double>("neutral_x_front", 3.5);
    neutral_x_back_   = declare_parameter<double>("neutral_x_back",  3.7);
    max_stride_x_     = declare_parameter<double>("max_stride_x",   2.0);
    max_stride_y_     = declare_parameter<double>("max_stride_y",   2.0);

    // IK-Geometrie – per Param überschreibbar (Meterskala aus deinem Repo als Referenz, hier relative Einheiten OK)
    upper_leg_length_     = declare_parameter<double>("upper_leg_length",     10.75); // cm
    lower_leg_length_     = declare_parameter<double>("lower_leg_length",     13.0);  // cm
    hip_body_distance_    = declare_parameter<double>("hip_body_distance",     5.5);  // cm

    // SMOV Publisher (genau wie in executable.h / walking_gait)
    front_pub_ = this->create_publisher<smov_msgs::msg::StatesServos>("front_proportional_servos", 10);
    back_pub_  = this->create_publisher<smov_msgs::msg::StatesServos>("back_proportional_servos", 10);
    front_servos_.state_name = "MOTION_CMD";
    back_servos_.state_name  = "MOTION_CMD";

    // IK State (benutzt Publisher + Servos + Längen)
    trig_ = std::make_unique<smov::TrigonometryState>(
      &front_servos_, &back_servos_, &front_pub_, &back_pub_,
      &upper_leg_length_, &lower_leg_length_, &hip_body_distance_
    );

    // Teleop
    keys_.init();

    // Timer Loop
    double dt = 1.0 / update_rate_hz_;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&MotionCmdNode::loop, this)
    );

    RCLCPP_INFO(get_logger(), "motion_cmd started (W=start, S=stop, arrows=move)");
  }

  ~MotionCmdNode() override { keys_.shutdown(); }

private:
  // --- Gait/Control state ---
  bool walking_{false};
  double phase_{0.0};
  double vx_{0.0}, vy_{0.0};      // vor/zurück, strafe links/rechts
  const double base_speed_x_ = 0.8;  // tastendruck -> gewünschte Schrittweite (relativ)
  const double base_speed_y_ = 0.8;

  // --- Params ---
  double update_rate_hz_;
  double gait_freq_hz_;
  double step_height_;
  double neutral_y_;
  double neutral_x_front_;
  double neutral_x_back_;
  double max_stride_x_;
  double max_stride_y_;
  double upper_leg_length_, lower_leg_length_, hip_body_distance_;

  // --- SMOV ---
  rclcpp::Publisher<smov_msgs::msg::StatesServos>::SharedPtr front_pub_, back_pub_;
  smov_msgs::msg::StatesServos front_servos_, back_servos_;
  std::unique_ptr<smov::TrigonometryState> trig_;

  // --- infra ---
  rclcpp::TimerBase::SharedPtr timer_;
  motion_cmd::TeleopKeys keys_;

  // Hilfskurve wie im walking_gait-Beispiel (halbkreisförmige Schrittbahn)
  float curved(float x, float dist_from_origin, float gap) {
    // -sqrt(r^2 - (2x - d)^2) + neutral_y + gap, r=5 → zu deinen Werten passend skalieren
    return -std::sqrt(std::max(0.0f, 25.0f - std::pow((2.0f*float(x) - float(dist_from_origin)), 2.0f)))
           + float(neutral_y_) + float(gap);
  }

  void loop() {
    // 1) Keyboard lesen
    if (auto k = keys_.readKey()) {
      const std::string &key = *k;
      if (key.size() == 1) {
        char c = std::tolower(key[0]);
        if (c == 'w') { walking_ = true;  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "WALK ON"); }
        if (c == 's') { walking_ = false; vx_ = 0.0; vy_ = 0.0; RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "WALK OFF"); }
        if (c == ' ') { vx_ = 0.0; vy_ = 0.0; }
      } else {
        if (key == "UP")    { vx_ =  base_speed_x_; }
        if (key == "DOWN")  { vx_ = -base_speed_x_; }
        if (key == "LEFT")  { vy_ = -base_speed_y_; }
        if (key == "RIGHT") { vy_ =  base_speed_y_; }
      }
    }

    // 2) Gait-Phase
    double dt = 1.0 / update_rate_hz_;
    if (walking_) {
      phase_ += dt * gait_freq_hz_;
      if (phase_ >= 1.0) phase_ -= 1.0;
    }

    // 3) Zielkoordinaten je Bein erstellen
    // Leg-Gruppen: (1,4) in Phase, (2,3) gegenphasig (klassischer Trot)
    // Annahme Leg-IDs: 1=FL, 2=FR, 3=BL, 4=BR
    smov::Vector3 fl, fr, bl, br;

    if (!walking_) {
      // Stand: neutrale Pose
      fl = {float(neutral_x_front_), float(neutral_y_), 5.0f};
      fr = {float(neutral_x_front_), float(neutral_y_), 5.0f};
      bl = {float(neutral_x_back_),  float(neutral_y_), 5.0f};
      br = {float(neutral_x_back_),  float(neutral_y_), 5.0f};
    } else {
      // einfache Schritttrajektorien
      const double stride_x = clampf(vx_, -max_stride_x_, max_stride_x_);
      const double stride_y = clampf(vy_, -max_stride_y_, max_stride_y_);
      const double ph0 = phase_;
      const double ph1 = std::fmod(phase_ + 0.5, 1.0);

      // Hilfsfunktion: aus Phase eine x-Position um neutral herum
      auto step_x = [&](double neutral_x, double ph, double stride)->float {
        // stance 0..0.5: Fuß "nach hinten", swing 0.5..1: Fuß nach vorn
        if (ph < 0.5) {
          double t = ph / 0.5; // 0..1
          return float(neutral_x - stride * (1.0 - t)); // von +stride nach 0
        } else {
          double t = (ph - 0.5) / 0.5; // 0..1
          return float(neutral_x + stride * t); // von 0 nach +stride
        }
      };
      // y (strafe) – analog, plus kleine „Kurve“ (gap) im Swing
      auto step_y = [&](double neutral_y, double ph, double stride)->float {
        double gap = (ph >= 0.5) ? 0.5 : 0.0; // Swing leicht anheben seitlich
        // symmetrisch um neutral_y; curved macht Bogenform
        return curved( float(stride), 2.0f, float(gap) );
      };
      // z hebt im Swing leicht an
      auto step_z = [&](double ph)->float {
        if (ph < 0.5) return 5.0f; // stance
        double t = (ph - 0.5) / 0.5; // 0..1
        return 5.0f + float(step_height_) * float(std::sin(float(M_PI)*t));
      };

      // Gruppe (FL,BR) = ph0
      fl.x = step_x(neutral_x_front_, ph0,  stride_x);
      fl.y = float(neutral_y_ + stride_y);
      fl.z = step_z(ph0);

      br.x = step_x(neutral_x_back_,  ph0,  stride_x);
      br.y = float(neutral_y_ + stride_y);
      br.z = step_z(ph0);

      // Gruppe (FR,BL) = ph1 (gegenphasig)
      fr.x = step_x(neutral_x_front_, ph1,  stride_x);
      fr.y = float(neutral_y_ + stride_y);
      fr.z = step_z(ph1);

      bl.x = step_x(neutral_x_back_,  ph1,  stride_x);
      bl.y = float(neutral_y_ + stride_y);
      bl.z = step_z(ph1);
    }

    // 4) IK aufrufen – publisht StatesServos auf /front_proportional_servos & /back_proportional_servos
    trig_->set_leg_to(1, fl); // FL
    trig_->set_leg_to(2, fr); // FR
    trig_->set_leg_to(3, bl); // BL
    trig_->set_leg_to(4, br); // BR
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto n = std::make_shared<MotionCmdNode>();
  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
