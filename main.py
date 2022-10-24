import numpy as np
import matplotlib.pyplot as plt
from PID import PID
import rospy

# constants
KP = 0.005
KI = 0.002
KD = 0.001
SETPOINTPOS = 1000  # mm
SETPOINTINCL = 0  # gradient angle
MINOUTPUT = -0.5  # more power to left wheel
MAXOUTPUT = 0.5  # more power to right wheel
JUMP = [780, 630, 600, 550]


pidPos = PID(KP, KI, KD, SETPOINTPOS, MINOUTPUT, MAXOUTPUT)
pidIncl = PID(KP, KI, KD, SETPOINTINCL, MINOUTPUT, MAXOUTPUT)


def dataProcessing():
    # measurement (subscribe to sensor) / y values
    # lidarDataRawY = [995.226667, 992.910667, 994.485333, 995.016]
    # lidarDataRawYInv = [lidarDataRawY[3], lidarDataRawY[2],lidarDataRawY[1], lidarDataRawY[0]]

    # FOV / x values
    lidarFovAngle = 25
    lidarFov = 2 * np.mean(JUMP) * np.sin(lidarFovAngle / 2 * np.pi / 180) / np.sin(77.5 * np.pi / 180)
    lidarDataRawX = [lidarFov * 1/8, lidarFov * 3/8, lidarFov * 5/8, lidarFov * 7/8]

    # Interpolation
    lidarPoly = np.poly1d(np.polyfit(lidarDataRawX, JUMP, 1))
    lidarPolyGradient = np.polyder(lidarPoly)
    lidarPolyGradientAngle = float(np.arctan(lidarPolyGradient) * 180 / np.pi)

    pidPosOutputValue = pidPos.compute(np.mean(JUMP))
    pidInclOutputValue = pidIncl.compute(lidarPolyGradientAngle)

    # debug
    print("pos: " + str(np.mean(JUMP)))
    print("fov: " + str(lidarFov))
    print("angle: " + str(lidarPolyGradientAngle))
    print("pidPos: " + str(pidPosOutputValue))
    print("pidIncl: " + str(pidInclOutputValue))

    # fig1
    plt.figure(1)
    plt.plot(lidarDataRawX, JUMP, 'o', color='b')
    plt.plot(lidarDataRawX, lidarPoly(lidarDataRawX), color='r')
    plt.title("Lidar Polynomial")
    plt.xlabel("Lidar FOV [mm]")
    plt.ylabel("Wall distance [mm]")
    plt.draw()
    plt.pause(0.001)


def main():
    while 1:
        dataProcessing()


if __name__ == "__main__":
    main()
