#!/usr/bin/env python

from scara_command.srv import ScaraKinFK, ScaraKinFKResponse
from math import atan2, sqrt, pow
import numpy
import rospy


## Assuming that the input is q1 q2 q3 the output should be x y z phi theta psi
def homogeneous_A(a1, alph1, d1, theta1):
    A1 = numpy.array([[numpy.cos(theta1), (numpy.sin(theta1)*-1) * numpy.cos(alph1), numpy.sin(theta1) * numpy.sin(alph1), a1 * numpy.cos(theta1)],
                      [numpy.sin(theta1), numpy.cos(theta1) * numpy.cos(alph1), (numpy.cos(theta1)*-1) * numpy.sin(alph1), a1 * numpy.sin(theta1)],
                      [0, numpy.sin(alph1), numpy.cos(alph1), d1],
                      [0, 0, 0, 1]])
    return A1


def handle_forward_kinematics(req):
    ##_---------------------- DH Parameters----------------------------------------
    a1 = .425
    alph1 = 0
    d1 = .55
    theta1 = req.q1

    a2 = .345
    alph2 = 3.14
    d2 = 0
    theta2 = req.q2

    a3 = 0
    alph3 = 0
    d3 = req.q3  + .11
    theta3 = 0

    A1 = homogeneous_A(a1, alph1, d1, theta1)
    A2 = homogeneous_A(a2, alph2, d2, theta2)
    A3 = homogeneous_A(a3, alph3, d3, theta3)

    transform = numpy.matmul(numpy.matmul(A1, A2), A3)

    x = transform[0, 3]
    y = transform[1, 3]
    z = transform[2, 3]
    roll = atan2(transform[2, 1], transform[2, 2])
    pitch = atan2((-1 * transform[2, 0]), sqrt(pow(transform[2, 1], 2) + pow(transform[2, 2], 2)))
    yaw = atan2(transform[1, 0], transform[0, 0])

    return ScaraKinFKResponse(x, y, z, roll, pitch, yaw)


def scara_FK_server():
    rospy.init_node('for_kin_server')
    s = rospy.Service('for_kin', ScaraKinFK, handle_forward_kinematics)

    print "Ready to perform FK"
    rospy.spin()


if __name__ == "__main__":
    scara_FK_server()
