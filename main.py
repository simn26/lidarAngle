import numpy as np
import matplotlib.pyplot as plt
from PID import PID

# constants
KP = 0.05
KI = 0.02
KD = 0.01
SETPOINT = 0  # gradient angle
MINOUTPUT = -0.5  # more power to left wheel
MAXOUTPUT = 0.5  # more power to right wheel


pid = PID(KP, KI, KD, SETPOINT, MINOUTPUT, MAXOUTPUT)


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

    pidOutputValue = pid.compute(lidarPolyGradientAngle)

    # debug
    print(np.mean(lidarDataRawY))
    print(lidarFov)
    print(lidarPolyGradientAngle)
    print(pidOutputValue)

    plt.figure(1)
    plt.plot(lidarDataRawX, lidarDataRawY, 'o')
    plt.plot(lidarDataRawX, lidarPoly(lidarDataRawX))
    plt.title("Interpolated Lidar Polynomial")
    plt.xlabel("Lidar FOV [mm]")
    plt.ylabel("Wall distance [mm]")
    #plt.show()


def main():
    while 1:
        dataProcessing()


if __name__ == "__main__":
    main()
