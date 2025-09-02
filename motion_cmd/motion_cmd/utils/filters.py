from dataclasses import dataclass

@dataclass
class RateLimitedFirstOrderFilter:
    tau: float
    dt: float
    max_rate: float
    y: float = 0.0

    def reset(self, val: float = 0.0):
        self.y = val

    def step(self, x: float) -> float:
        alpha = self.dt / max(self.tau, 1e-6)
        y_target = self.y + alpha * (x - self.y)
        max_step = self.max_rate * self.dt
        dy = max(-max_step, min(max_step, y_target - self.y))
        self.y += dy
        return self.y
