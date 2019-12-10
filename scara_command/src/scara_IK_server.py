#!/usr/bin/env python
import rospy
import math
from scara_command.srv import ScaraKinIK, ScaraKinIKResponse
#Assuming that input is x,y,z,phi,theta,psi and output is q1,q2,q3


def handle_IK(req):
    # Robot dimension
    a1 = 0.55
    a2 = 0.425
    a3 = 0.345
    a4 = 0.11

    # Input Coordinate
    x = req.x
    y = req.y
    z = req.z
    psi = req.psi
	
    # Perform IK
    q3 = a1 - a4 - z
    q2 = math.acos(round(-(a2**2 + a3**2 - x**2 - y**2) / (2*a2*a3),3))
    q2_1 = q2
    q2_2 = -q2
    q1_1 = psi-q2_1
    q1_2 = psi-q2_2

    #Checking with forward kinematics which q1,q2 pair is correct
    fwd_x = a2*math.cos(q1_1)+a3*math.cos(q1_1+q2_1)
    fwd_y = a2*math.sin(q1_1)+a3*math.sin(q1_1+q2_1)
    if abs(fwd_x-x)<0.01 and abs(fwd_y-y)<0.01:
       q1 = q1_1
       q2 = q2_1
    else:
       q1 = q1_2
       q2 = q2_2

    return ScaraKinIKResponse(q1, q2, q3)

def scara_IK_server():
    rospy.init_node('scara_IK_server')
    s = rospy.Service('inv_kin', ScaraKinIK, handle_IK)
    
    rospy.spin()


if __name__ == "__main__":
    scara_IK_server()

