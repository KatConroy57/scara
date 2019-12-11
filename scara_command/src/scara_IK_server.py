#!/usr/bin/env python
import rospy
from math import sin, cos, acos, atan2
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
    try:
        # calculate joint 3
        q3 = a1 - a4 - z
        if not (0 <= q3 <= 0.3):
            print "IK failed, coordinate z is not reachable."
            return ScaraKinIKResponse(False, 0, 0, 0)
        # calculate joint 2
        q2 = acos(round(-(a2**2 + a3**2 - x**2 - y**2) / (2*a2*a3),3))
        q2_1 = q2
        q2_2 = -q2
        # calculate joint 1
        nor_1 = pow((a3*cos(q2_1)+a2), 2) + pow(a3*sin(q2_1) , 2)
        nor_2 = pow((a3*cos(q2_2)+a2), 2) + pow(a3*sin(q2_2) , 2)
        c1_1 = (x*(a3*cos(q2_1) + a2) + y*a3*sin(q2_1)) / nor_1
        c1_2 = (x*(a3*cos(q2_2) + a2) + y*a3*sin(q2_2)) / nor_2
        s1_1 = (y*(a3*cos(q2_1) + a2) - x*a3*sin(q2_1)) / nor_1
        s1_2 = (y*(a3*cos(q2_2) + a2) - x*a3*sin(q2_2)) / nor_2
        q1_1 = atan2(s1_1, c1_1)
        q1_2 = atan2(s1_2, c1_2)
        
        # Checking which q1, q2 pair is correct
        # Second set of solution
        calculated_psi = q2_2 + q1_2
        if abs(calculated_psi-psi) < 0.01:
            q1 = q1_2
            q2 = q2_2
        else:
            q1 = q1_1
            q2 = q2_1
    except ValueError, e:
        print "IK failed, the coordinate x, y provided may be invalid: %s"%e
        return ScaraKinIKResponse(False, 0, 0, 0)

    return ScaraKinIKResponse(True, q1, q2, q3)


def scara_IK_server():
    rospy.init_node('scara_IK_server')
    s = rospy.Service('inv_kin', ScaraKinIK, handle_IK)
    
    rospy.spin()


if __name__ == "__main__":
    scara_IK_server()