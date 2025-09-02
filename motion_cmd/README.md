# motion_cmd (ROS 2, Python)

Ein ROS 2-Python-Paket, das einen **Spot-Micro-ähnlichen** Motion-Commander als eigenständigen Node bereitstellt.
Es ist als Vorlage gedacht, die sich in das **SMOV**-Ökosystem (i2c_pwm_board, smov_config, smov_lib, smov_states)
einfügt, aber **unabhängig vom Paket `walking_gait`** arbeitet.

**Hauptmerkmale**

- Zustandsmaschine: *idle*, *stand*, *walk* (ähnlich zu `spot_micro_motion_cmd` aus dem Referenzprojekt).
- Kommandos: `/stand_cmd`, `/idle_cmd`, `/walk_cmd` (`std_msgs/Bool`), `/angle_cmd` (`geometry_msgs/Vector3`), `/cmd_vel` (`geometry_msgs/Twist`).
- Dual-Board-Unterstützung über zwei Topics (z. B. `/servos_proportional_1` und `/servos_proportional_6`) vom Paket **i2c_pwm_board**.
- Optional: IMU-Einbindung über `/imu/data` (`sensor_msgs/Imu`) – kann per Parameter aktiviert/deaktiviert werden.
- Parametrisierbarer *Gait* (Vorlage in `motion_cmd/gaits/dual_board_gait.py`).

> **Hinweis:** Diese Vorlage sendet standardmäßig Servo-Proportionalwerte in [-1, 1] an `i2c_pwm_board`. 
> Die tatsächliche Winkelkalibrierung (Center/Range/Direction) erfolgt wie gewohnt **im i2c_pwm_board** selbst
> (per Services/Kalibrier-Tools). Alternativ können Sie die Ausgabe auf SMOV-States umschalten.

Siehe `launch/motion_cmd.launch.py` und `motion_cmd/params/motion_cmd.yaml` für eine Beispielkonfiguration.
