# motion_cmd (ROS 2 Jazzy, Python)

Eigenständiger **Motion Commander** für Spot-Micro/SMOV unter **ROS 2 Jazzy** –
orientiert an `spot_master`/`spotMicro`, **ohne LCD** und **ohne `walking_gait`**.

**Features**
- Dual-Board (PCA9685): **Front = Bus 6**, **Back = Bus 4** (Default).
- Ausgabe an **i2c_pwm_board**: `/servos_proportional_<bus>` (Proportional in [-1,1]).
- Optionaler **SMOV-States**-Modus (per Param), keine harte Kopplung.
- **BNO085**-IMU per Param (`use_imu`, `imu_topic`), Default: `/bno085/imu`.
- Kommandos wie Spot-Micro: `/stand_cmd`, `/idle_cmd`, `/walk_cmd`, `/angle_cmd`, `/cmd_vel`.
- Gait mit **vorwärts/rückwärts**, **links/rechts (strafe)** und **drehen**.

## Build (Jazzy)
```bash
cd ~/ros2_ws/src
unzip motion_cmd_jazzy_full.zip
cd ..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select motion_cmd
source install/setup.bash
```

## Run
```bash
# i2c_pwm_board (je Bus eine Node)
ros2 run i2c_pwm_board node 6   # Front
ros2 run i2c_pwm_board node 4   # Back

# IMU-Node aus bno085_imu_py (publisht sensor_msgs/Imu auf /bno085/imu)

# motion_cmd starten
ros2 launch motion_cmd motion_cmd.launch.py
```

## Steuerung
```bash
ros2 topic pub /stand_cmd std_msgs/Bool "data: true" -1
ros2 topic pub /walk_cmd  std_msgs/Bool "data: true" -1
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0}, angular: {z: 0.0}}" -1   # vorwärts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x:-0.1, y: 0.0}, angular: {z: 0.0}}" -1   # rückwärts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.1}, angular: {z: 0.0}}" -1   # strafe links
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y:-0.1}, angular: {z: 0.0}}" -1   # strafe rechts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.3}}" -1   # links drehen
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z:-0.3}}" -1   # rechts drehen
ros2 topic pub /idle_cmd  std_msgs/Bool "data: true" -1
```

## Parameter (siehe `params/motion_cmd.yaml`)
- `front_bus: 6`, `back_bus: 4`
- `channels_front`, `channels_back` (6 Kanäle je Board)
- `use_imu: true`, `imu_topic: /bno085/imu`
- `output_mode: pwm|states` (pwm = i2c_pwm_board; states = SMOV-Topics)
- `proportional_scale`: globale Skalierung der (−1..1)-Ausgabe
