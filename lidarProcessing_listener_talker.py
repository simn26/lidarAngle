import numpy as np
import matplotlib.pyplot as plt

from PID import PID

#import rospy
#from std_msgs.msg import Float32MultiArray


# constants
#     Pos     Inc
KP = [0.005, 0.005]
KI = [0.002, 0.002]
KD = [0.001, 0.001]

SETPOINTPOS = 1000  # mm
SETPOINTINCL = 0  # gradient angle
MINOUTPUT = -50  # more power to left wheel
MAXOUTPUT = 50  # more power to right wheel


pidPos = PID(KP[0], KI[0], KD[0], SETPOINTPOS, MINOUTPUT, MAXOUTPUT)
pidIncl = PID(KP[1], KI[1], KD[1], SETPOINTINCL, MINOUTPUT, MAXOUTPUT)


def dataProcessing(lidarDataRawY):
    #pub

    # measurement (subscribe to sensor) / y values
    lidarDataRawYInv = [lidarDataRawY[3], lidarDataRawY[2],lidarDataRawY[1], lidarDataRawY[0]]

    # FOV / x values
    lidarFovAngle = 25
    lidarFov = 2 * np.mean(lidarDataRawYInv) * np.sin(lidarFovAngle / 2 * np.pi / 180) / np.sin(77.5 * np.pi / 180)
    lidarDataRawX = [lidarFov * 1/8, lidarFov * 3/8, lidarFov * 5/8, lidarFov * 7/8]

    # Interpolation
    lidarPoly = np.poly1d(np.polyfit(lidarDataRawX, lidarDataRawYInv, 1))
    lidarPolyGradient = np.polyder(lidarPoly)
    lidarPolyGradientAngle = float(np.arctan(lidarPolyGradient) * 180 / np.pi)

    pidPosOutputValue = pidPos.compute(np.mean(lidarDataRawYInv))
    pidInclOutputValue = pidIncl.compute(lidarPolyGradientAngle)

    # broadcast control value
    if SETPOINTPOS - 10 <= np.mean(lidarDataRawYInv) <= SETPOINTPOS + 10:
        print("inside threshold")
        #pub.publisch(pidPosOutputValue * 0.5 + pidInclOutputValue)
        print("pidPos: " + str(pidPosOutputValue) + " pidIncl: " + str(pidInclOutputValue))
        print("pidPublish: " + str(pidPosOutputValue * 0.5 + pidInclOutputValue))
    else:
        print("outside threshold")
        #pub.publisch(pidPosOutputValue + pidInclOutputValue * 0.5)
        print("pidPos: " + str(pidPosOutputValue) + " pidIncl: " + str(pidInclOutputValue))
        print("pidPublish: " + str(pidPosOutputValue + pidInclOutputValue * 0.5))

    print("pos: " + str(np.mean(lidarDataRawYInv)))
    print("angle: " + str(lidarPolyGradientAngle))
    print("fov: " + str(lidarFov))


    # fig1
    plt.figure(1)
    plt.plot(lidarDataRawX, lidarDataRawYInv, 'o', color='b')
    plt.plot(lidarDataRawX, lidarPoly(lidarDataRawX), color='r')
    plt.title("Lidar Polynomial")
    plt.xlabel("Lidar FOV [mm]")
    plt.ylabel("Wall distance [mm]")
    plt.draw()
    plt.pause(0.001)


# def listener():
#     rospy.init_node("lidarProcessing_listener_talker", anonymous=False)
#     rospy.Subscriber("VL53L0X_talker", Float32MulitArray, dataProcessing)
#     print("Listener started...")
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()


if __name__ == "__main__":
    #while 1:
        for i in range(0, 220):
            dataProcessing([780.0, 780.0, 780.0, 1000.0])
    # try:
    #     pub = rospy.Publisher("lidarProcessing_listener_talker", float, queue_size=10)
    #     print("start publisher node")
    #     listener()
    # except rospy.ROSInterruptException:
    #     print('ROSInterruptException')
    #     pass
    # finally:
    #     print("close")
