from dataclasses import dataclass
import math
from typing import List

@dataclass
class GaitParams:
    dt: float
    stand_height_m: float
    body_length_m: float
    body_width_m: float
    link_hip_m: float
    link_upper_m: float
    link_lower_m: float
    step_frequency_hz: float
    step_height: float
    max_stride: float

class DualBoardTrotGait:
    """Trot-Gait mit vor/zurück, drehen und **echtem Strafen** (links/rechts).
    Output: 12 proportional targets [-1,1], Order: FL[abd,hip,knee], FR..., BL..., BR...
    """
    def __init__(self, params: GaitParams, scale: float = 0.5):
        self.p = params
        self.scale = max(0.0, min(1.0, scale))
        self.phase = 0.0

    def update(self, vx: float, vy: float, wz: float, roll_off: float, pitch_off: float) -> List[float]:
        self.phase = (self.phase + self.p.dt * self.p.step_frequency_hz) % 1.0

        legs = ['FL','FR','BL','BR']
        ph = {
            'FL': self.phase,
            'BR': self.phase,
            'FR': (self.phase + 0.5) % 1.0,
            'BL': (self.phase + 0.5) % 1.0,
        }

        def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

        stride_fwd = clamp(vx * 0.5, -self.p.max_stride, self.p.max_stride)
        stride_lat = clamp(vy * 0.5, -self.p.max_stride, self.p.max_stride)
        turn = clamp(wz * 0.2, -self.p.max_stride, self.p.max_stride)

        out = []
        for L in legs:
            side = +1.0 if L in ['FL','BL'] else -1.0  # + links, - rechts
            s = math.sin(2*math.pi*ph[L])

            abd = self.scale * clamp( 0.30*stride_lat*side - 0.15*roll_off, -1.0, 1.0)
            hip_cmd = 0.35*s + 0.25*stride_fwd + 0.20*(turn if L in ['FL','BR'] else -turn) - 0.15*pitch_off
            hip = self.scale * clamp(hip_cmd, -1.0, 1.0)
            knee = self.scale * clamp(-0.35*s, -1.0, 1.0)

            out.extend([abd, hip, knee])
        return out

    def stand(self, roll_off: float, pitch_off: float) -> List[float]:
        abd = self.scale * (-0.1 * roll_off)
        hip = self.scale * (-0.05 * pitch_off)
        knee = 0.0
        return [abd, hip, knee] * 4

    def idle(self) -> List[float]:
        return [0.0] * 12
