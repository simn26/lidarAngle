import time


# PID test class
class PID:
    def __init__(self, kp, ki, kd, setpoint, minO, maxO):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.minO = minO
        self.maxO = maxO
        self.error = 0
        self.errSum = 0
        self.dErr = 0
        self.lastErr = 0
        self.sampleTime = 0.001
        self.timeChange = 0
        self.lastTime = 0
        self.output = 0.0

    def compute(self, actualValue):
        now = round(time.time() * 1000)
        self.timeChange = now - self.sampleTime

        if self.timeChange >= self.sampleTime:
            self.error = self.setpoint - actualValue
            self.errSum += self.error
            self.dErr = self.error - self.lastErr

            self.output = self.kp * self.error + self.ki * self.errSum + self.kd * self.dErr

            self.lastErr = self.error
            self.lastTime = now

            if self.output < self.minO:
                self.output = self.minO
            if self.output > self.maxO:
                self.output = self.maxO
            return self.output
