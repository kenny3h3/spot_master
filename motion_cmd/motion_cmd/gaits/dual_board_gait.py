from dataclasses import dataclass
import math
from typing import Tuple, List

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
    """Minimal, ROS-agnostische Gait-Vorlage.
    - Input: vx, vy, wz body command + optional body pitch/roll offsets
    - Output: 12 **proportional** servo targets in [-1,1] (Abstraktion)
      Order: FL [abd, hip, knee], FR [...], BL [...], BR [...]
    """
    def __init__(self, params: GaitParams, scale: float = 0.5):
        self.p = params
        self.scale = max(0.0, min(1.0, scale))
        self.phase = 0.0

    def update(self, vx: float, vy: float, wz: float, roll_off: float, pitch_off: float) -> List[float]:
        # Fortschritt der Schrittphase
        self.phase = (self.phase + self.p.dt * self.p.step_frequency_hz) % 1.0
        # Sehr einfache Platzhalter-Kinematik: Sinusförmiger Hüft-/Knieverlauf pro Bein
        # Trot: FL & BR in Phase, FR & BL gegenphasig
        legs = ['FL','FR','BL','BR']
        ph = {
            'FL': self.phase,
            'BR': self.phase,
            'FR': (self.phase + 0.5) % 1.0,
            'BL': (self.phase + 0.5) % 1.0,
        }
        # Ableitung der Kommandos in hip/knee-Offsets – rein heuristisch
        stride = max(-self.p.max_stride, min(self.p.max_stride, vx * 0.5))
        turn = max(-self.p.max_stride, min(self.p.max_stride, wz * 0.2))
        # Ausgabepuffer
        out = []
        for L in legs:
            # Abduktion (Seite) leicht mit vy koppeln
            abd = self.scale * max(-1.0, min(1.0, vy * 0.5))
            # Hüfte/Knie: simple Heben/Senken
            s = math.sin(2*math.pi*ph[L])
            c = math.cos(2*math.pi*ph[L])
            hip = self.scale * (0.3 * s + 0.2 * stride + 0.2 * (turn if L in ['FL','BR'] else -turn))
            knee = self.scale * (-0.3 * s)
            # roll/pitch Offsets (sehr reduziert)
            hip += self.scale * ( -0.15 * pitch_off )
            abd += self.scale * ( -0.15 * roll_off )
            out.extend([max(-1.0,min(1.0,abd)), max(-1.0,min(1.0,hip)), max(-1.0,min(1.0,knee))])
        return out

    def stand(self, roll_off: float, pitch_off: float) -> List[float]:
        # neutrale Pose, etwas Beinstellung
        abd = self.scale * (-0.1 * roll_off)
        hip = self.scale * (-0.05 * pitch_off)
        knee = 0.0
        return [abd, hip, knee] * 4

    def idle(self) -> List[float]:
        return [0.0] * 12
