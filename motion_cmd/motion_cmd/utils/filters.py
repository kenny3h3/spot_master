from dataclasses import dataclass

@dataclass
class RateLimitedFirstOrderFilter:
    tau: float      # time constant [s]
    dt: float       # sample time [s]
    max_rate: float # max slew rate per second (abs)
    y: float = 0.0

    def reset(self, val: float = 0.0):
        self.y = val

    def step(self, x: float) -> float:
        # simple FO filter with rate limit
        alpha = self.dt / max(self.tau, 1e-6)
        y_target = self.y + alpha * (x - self.y)
        max_step = self.max_rate * self.dt
        dy = y_target - self.y
        if dy > max_step:
            dy = max_step
        elif dy < -max_step:
            dy = -max_step
        self.y += dy
        return self.y
