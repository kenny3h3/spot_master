#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <cmath>
#include "motion_cmd/teleop_keys.hpp"

class TeleopKbNode : public rclcpp::Node {
public:
  TeleopKbNode() : Node("teleop_kb") {
    // Parameter
    rate_hz_      = declare_parameter<double>("rate_hz", 50.0);
    lin_speed_    = declare_parameter<double>("linear_speed", 0.6);   // m/s (gewünschte Skalierung)
    ang_speed_    = declare_parameter<double>("angular_speed", 0.8);  // rad/s
    accel_        = declare_parameter<double>("accel", 3.0);          // sanftes Rampen
    decel_        = declare_parameter<double>("decel", 4.0);

    pub_cmd_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_walk_ = create_publisher<std_msgs::msg::Bool>("/walk_cmd", 10);
    pub_idle_ = create_publisher<std_msgs::msg::Bool>("/idle_cmd", 10);

    if (!keys_.init()) {
      RCLCPP_WARN(get_logger(), "Kein TTY-Rawmode: über SSH nutze `ssh -t -t ...` oder steuere per Topics.");
    } else {
      RCLCPP_INFO(get_logger(), "Tastatur aktiv: ↑/↓ vor/zurück, ←/→ drehen, S/Space = Stopp");
    }

    timer_ = create_wall_timer(std::chrono::milliseconds((int)(1000.0/rate_hz_)),
                               std::bind(&TeleopKbNode::loop, this));
  }

  ~TeleopKbNode() override { keys_.shutdown(); }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_walk_, pub_idle_;
  rclcpp::TimerBase::SharedPtr timer_;
  motion_cmd::TeleopKeys keys_;

  double rate_hz_, lin_speed_, ang_speed_, accel_, decel_;
  double vx_target_{0.0}, wz_target_{0.0};
  double vx_{0.0}, wz_{0.0};

  static double ramp(double cur, double tgt, double up, double down, double dt) {
    double diff = tgt - cur;
    double maxstep = ( (diff>0) ? up : down ) * dt;
    if (std::fabs(diff) <= std::fabs(maxstep)) return tgt;
    return cur + std::copysign(maxstep, diff);
  }

  void loop() {
    // Eingabe lesen (wahlweise mehrmals pro Tick)
    for (int i=0;i<4;i++) {
      auto k = keys_.readKey();
      if (!k) break;
      handle_key(*k);
    }

    // Rampen
    double dt = 1.0 / rate_hz_;
    vx_ = ramp(vx_, vx_target_, accel_, decel_, dt);
    wz_ = ramp(wz_, wz_target_, accel_, decel_, dt);

    // Publizieren
    geometry_msgs::msg::Twist t;
    t.linear.x  = vx_;
    t.angular.z = wz_;
    pub_cmd_->publish(t);

    // Walk/Idle je nach Bewegung
    const bool moving = (std::fabs(vx_) > 1e-3) || (std::fabs(wz_) > 1e-3);
    std_msgs::msg::Bool b;
    if (moving) { b.data = true;  pub_walk_->publish(b); }
    else        { b.data = true;  pub_idle_->publish(b); }
  }

  void handle_key(const std::string& key) {
    // Stop
    if ((key.size()==1 && (key[0]=='s' || key[0]=='S' || key[0]==' '))) {
      vx_target_ = 0.0; wz_target_ = 0.0; return;
    }
    // Pfeile
    if      (key=="UP")    { vx_target_ = +lin_speed_; /* wz bleibt */ }
    else if (key=="DOWN")  { vx_target_ = -lin_speed_; /* wz bleibt */ }
    else if (key=="LEFT")  {
      // wenn kein Vor/Rück -> reines Drehen; sonst Kurve
      wz_target_ = +ang_speed_;
      if (std::fabs(vx_target_) < 1e-6) vx_target_ = 0.0;
    }
    else if (key=="RIGHT") {
      wz_target_ = -ang_speed_;
      if (std::fabs(vx_target_) < 1e-6) vx_target_ = 0.0;
    }
    // Buchstaben w/a/d optional:
    else if (key=="w" || key=="W") { vx_target_ = +lin_speed_; }
    else if (key=="a" || key=="A") { wz_target_ = +ang_speed_; if (std::fabs(vx_target_)<1e-6) vx_target_=0.0; }
    else if (key=="d" || key=="D") { wz_target_ = -ang_speed_; if (std::fabs(vx_target_)<1e-6) vx_target_=0.0; }
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopKbNode>());
  rclcpp::shutdown();
  return 0;
}
