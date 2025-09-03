#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Imu
from i2c_pwm_board_msgs.msg import ServoArray, Servo

from dataclasses import dataclass
from enum import Enum
from typing import List
import math

# Optional: SMOV states publishing (import guarded – falls nicht vorhanden, bleibt PWM)
try:
    from smov_msgs.msg import FloatArray as SmovFloatArray  # ggf. an Typ anpassen
    _HAS_SMOV = True
except Exception:
    _HAS_SMOV = False

from .gaits.dual_board_gait import DualBoardTrotGait, GaitParams

class State(Enum):
    IDLE = 0
    STAND = 1
    WALK = 2

@dataclass
class BodyCmd:
    roll: float = 0.0
    pitch: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

class MotionCmdNode(Node):
    def __init__(self):
        super().__init__('motion_cmd')
        self.declare_parameters('', [
            ('update_rate_hz', 100.0),
            ('use_imu', True),
            ('imu_topic', '/bno085/imu'),
            ('output_mode', 'pwm'),  # 'pwm' or 'states'
            ('front_bus', 6),        # Front
            ('back_bus', 4),         # Back
            ('front_state_topic', '/smov/front_state'),
            ('back_state_topic', '/smov/back_state'),
            ('proportional_scale', 0.5),
            ('channels_front', [0,1,2,13,14,15]),
            ('channels_back',  [0,1,2,13,14,15]),
        ])

        self.state = State.IDLE
        self.cmd = BodyCmd()
        self.imu_enabled = bool(self.get_parameter('use_imu').value)
        self.last_yaw = 0.0

        # Gait
        dt = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.gait = DualBoardTrotGait(GaitParams(
            dt=dt,
            stand_height_m=0.155,
            body_length_m=0.186,
            body_width_m=0.078,
            link_hip_m=0.055,
            link_upper_m=0.1075,
            link_lower_m=0.130,
            step_frequency_hz=1.8,
            step_height=0.04,
            max_stride=0.08,
        ), scale=float(self.get_parameter('proportional_scale').value))

        # PWM publishers (i2c_pwm_board)
        fb = int(self.get_parameter('front_bus').value)
        bb = int(self.get_parameter('back_bus').value)
        self.pub_front_pwm = self.create_publisher(ServoArray, f'/servos_proportional_{fb}', 10)
        self.pub_back_pwm  = self.create_publisher(ServoArray, f'/servos_proportional_{bb}', 10)

        # Optional SMOV state pubs
        self.pub_front_state = None
        self.pub_back_state = None
        if str(self.get_parameter('output_mode').value) == 'states' and _HAS_SMOV:
            fst = str(self.get_parameter('front_state_topic').value)
            bst = str(self.get_parameter('back_state_topic').value)
            self.pub_front_state = self.create_publisher(SmovFloatArray, fst, 10)
            self.pub_back_state  = self.create_publisher(SmovFloatArray, bst, 10)
            self.get_logger().info('Output mode: SMOV states')
        else:
            self.get_logger().info('Output mode: PWM (i2c_pwm_board)')

        # Channel mapping per board
        self.channels_front = list(self.get_parameter('channels_front').value)
        self.channels_back  = list(self.get_parameter('channels_back').value)

        # Subscribers
        self.create_subscription(Bool, '/stand_cmd', self.on_stand, 10)
        self.create_subscription(Bool, '/idle_cmd',  self.on_idle,  10)
        self.create_subscription(Bool, '/walk_cmd',  self.on_walk,  10)
        self.create_subscription(Vector3, '/angle_cmd', self.on_angle, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_twist, 10)

        if self.imu_enabled:
            imu_topic = str(self.get_parameter('imu_topic').value)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Imu, imu_topic, self.on_imu, qos)
            self.get_logger().info(f'IMU enabled on {imu_topic}')

        self.timer = self.create_timer(dt, self.loop)
        self.get_logger().info('motion_cmd node started (dual board).')

    # --- Callbacks ---
    def on_stand(self, msg: Bool):
        if msg.data:
            self.state = State.STAND
            self.get_logger().info('State -> STAND')

    def on_idle(self, msg: Bool):
        if msg.data:
            self.state = State.IDLE
            self.get_logger().info('State -> IDLE')

    def on_walk(self, msg: Bool):
        if msg.data:
            self.state = State.WALK
            self.get_logger().info('State -> WALK')

    def on_angle(self, msg: Vector3):
        self.cmd.roll  = float(msg.x)
        self.cmd.pitch = float(msg.y)

    def on_twist(self, msg: Twist):
        self.cmd.vx = float(msg.linear.x)
        self.cmd.vy = float(msg.linear.y)
        self.cmd.wz = float(msg.angular.z)

    def on_imu(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.last_yaw = math.atan2(siny_cosp, cosy_cosp)

    # --- Main loop ---
    def loop(self):
        if self.state == State.IDLE:
            targets = self.gait.idle()
        elif self.state == State.STAND:
            targets = self.gait.stand(self.cmd.roll, self.cmd.pitch)
        else:
            targets = self.gait.update(self.cmd.vx, self.cmd.vy, self.cmd.wz, self.cmd.roll, self.cmd.pitch)

        # Split front/back
        front_vals = targets[:6]
        back_vals  = targets[6:]

        if self.pub_front_state and self.pub_back_state:
            # SMOV states (FloatArray – exemplarisch 6 Werte je Board)
            msg_f = SmovFloatArray(); msg_f.data = [float(v) for v in front_vals]
            msg_b = SmovFloatArray(); msg_b.data = [float(v) for v in back_vals]
            self.pub_front_state.publish(msg_f)
            self.pub_back_state.publish(msg_b)
        else:
            # PWM an i2c_pwm_board
            self.pub_front_pwm.publish(self._build_servo_array(front_vals, use_front=True))
            self.pub_back_pwm.publish(self._build_servo_array(back_vals,  use_front=False))

    def _build_servo_array(self, six_vals: List[float], use_front: bool) -> ServoArray:
        arr = ServoArray()
        mapping = self.channels_front if use_front else self.channels_back
        for i, v in enumerate(six_vals):
            chan = mapping[i] if i < len(mapping) else i
            s = Servo()
            s.servo = int(chan)
            s.value = float(max(-1.0, min(1.0, v)))
            arr.servos.append(s)
        return arr

def main():
    rclpy.init()
    node = MotionCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
