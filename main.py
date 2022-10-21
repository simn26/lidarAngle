import numpy as np
import matplotlib.pyplot as plt
import time

# constants
KP = 1.0
KI = 0.2
KD = 0.1
# regulate on 0° gradient angle
SETPOINT = 0


class PID(object):
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.error = 0
        self.errSum = 0
        self.dErr = 0
        self.lastErr = 0
        self.sampleTime = 0.001
        self.timeChange = 0
        self.lastTime = 0
        self.output = 0

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

            if self.output < -255:
                self.output = -255
            if self.output > 255:
                self.output = 255
            return int(self.output)


pid = PID(KP, KI, KD, SETPOINT)


def dataProcessing():
    # measurement (subscribe to sensor)
    lidarDataRawY = [995.226667, 992.910667, 994.485333, 995.016 + 200]

    # FOV / x values
    lidarFovAngle = 25
    lidarFov = 2 * np.mean(lidarDataRawY) * np.sin(lidarFovAngle / 2 * np.pi / 180) / np.sin(77.5 * np.pi / 180)
    lidarDataRawX = [lidarFov * 1/8, lidarFov * 3/8, lidarFov * 5/8, lidarFov * 7/8]

    # Interpolation (creating a line)
    lidarPoly = np.poly1d(np.polyfit(lidarDataRawX, lidarDataRawY, 1))
    lidarPolyGradient = np.polyder(lidarPoly)
    lidarPolyGradientAngle = float(np.arctan(lidarPolyGradient) * 180 / np.pi)

    # pass the value to the motors (-255 - 255)
    # negative integer = slow down right wheel
    # positive integer = slow down left wheel
    pid.compute(lidarPolyGradientAngle)

    # debug
    print(np.mean(lidarDataRawY))
    print(lidarFov)
    print(lidarPolyGradientAngle)

    fig = plt.figure(1)
    plt.plot(lidarDataRawX, lidarDataRawY, 'o')
    plt.plot(lidarDataRawX, lidarPoly(lidarDataRawX))
    plt.title("Interpolated Lidar Polynom")
    plt.xlabel("Lidar FOV [mm]")
    plt.ylabel("Abstand Wand [mm]")
    plt.show()


def main():
    while 1:
        dataProcessing()


if __name__ == "__main__":
    main()
