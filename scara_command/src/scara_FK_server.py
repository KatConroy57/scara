#!/usr/bin/env python

from scara_command.srv import ScaraKinFK, ScaraKinFKResponse, ScaraHomoMatrix
from math import atan2, sqrt, pow
import numpy
from helper import ma2np

import rospy


def handle_forward_kinematics(req):
    # Get Homogeneous Matrix
    rospy.wait_for_service('homogeneous_matrix')
    try:
        get_homo_matrix = rospy.ServiceProxy('homogeneous_matrix', ScaraHomoMatrix)
        homo_matrix = get_homo_matrix(req.q1, req.q2, req.q3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    A1 = ma2np(homo_matrix.A1)
    A2 = ma2np(homo_matrix.A2)
    A3 = ma2np(homo_matrix.A3)

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

    rospy.spin()


if __name__ == "__main__":
    scara_FK_server()
