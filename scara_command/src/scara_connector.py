#!/usr/bin/env python

# Import user defined scara srv
from scara_command.srv import ScaraKinFK, ScaraKinFKResponse,\
                              ScaraKinIK, ScaraKinIKResponse,\
                              CheckKinFK, CheckKinFKResponse,\
                              CheckKinIK, CheckKinIKResponse
from helper import acquire_coordinates, acquire_joints
import rospy
import math


# Check if two eular angles are the same
def check_ang(a1, a2):
    same = False
    # convert to between 0 and 2pi
    a1 = a1 % 6.283 
    a2 = a2 % 6.283
    if abs(a1-a2) < 0.01:
	same = True
    # To close the loop 0 -> 2pi -> 0
    if a1 < 0.01 or a2 < 0.01 :
	if abs(a1+a2-6.283) < 0.01:
	    same = True
    return same


# Check if IK server is correct
def handle_check_ik(req):
    # Acquire robot state from gazebo
    gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi = acquire_coordinates()
    gaze_q1, gaze_q2, gaze_q3 = acquire_joints()

    # Using inv_kin service to calculate joint values
    rospy.wait_for_service('inv_kin')
    try:
        inv_kinematic = rospy.ServiceProxy('inv_kin', ScaraKinIK)
        res = inv_kinematic(gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    q1, q2, q3 = res.q1, res.q2, res.q3

    # Judge
    correct = False
    if check_ang(q1,gaze_q1) and check_ang(q2,gaze_q2) and abs(q3-gaze_q3)<0.01:
	correct = True

    # return both results to compare
    print "Check IK success: ", correct
    return CheckKinIKResponse(gaze_q1, gaze_q2, gaze_q3, q1, q2, q3, correct)


# Check if FK server is correct
def handle_check_fk(req):
    # Acquire robot state from gazebo
    gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi = acquire_coordinates()
    gaze_q1, gaze_q2, gaze_q3 = acquire_joints()

    # Using for_kin service to calculate end-of-effector coordinate
    rospy.wait_for_service('for_kin')
    try:
        for_kinematic = rospy.ServiceProxy('for_kin', ScaraKinFK)
        res = for_kinematic(gaze_q1, gaze_q2, gaze_q3)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    x, y, z, phi, theta, psi = res.x, res.y, res.z, res.phi, res.theta, res.psi

    # Judge
    correct = False
    if abs(x-gaze_x)<0.01 and abs(y-gaze_y)<0.01 and abs(z-gaze_z)<0.01 and \
       check_ang(phi,gaze_phi) and check_ang(theta,gaze_theta) and check_ang(psi,gaze_psi):
	correct = True

    # return both results to compare
    print "Check FK success: ", correct
    return CheckKinFKResponse(gaze_x, gaze_y, gaze_z, gaze_phi, gaze_theta, gaze_psi, 
                            x, y, z, phi, theta, psi, correct)


# Just to compute ik
def handle_compute_ik(req):
    rospy.wait_for_service('inv_kin')
    try:
        inv_kinematic = rospy.ServiceProxy('inv_kin', ScaraKinIK)
	res = inv_kinematic(req.x, req.y, req.z, req.phi, req.theta, req.psi)
        print "IK response: ", res.q1,res.q2,res.q3
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    

# Just to compute fk
def handle_compute_fk(req):
    rospy.wait_for_service('for_kin')
    try:
        for_kinematic = rospy.ServiceProxy('for_kin', ScaraKinFK)
	res = for_kinematic(req.q1, req.q2, req.q3)
	print "FK response: ", res.x,res.y,res.z,res.phi,res.theta,res.psi
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    

# Connector services
def scara_connector():
    rospy.init_node('kin_connector')

    s1 = rospy.Service('check_ik', CheckKinIK, handle_check_ik)
    s2 = rospy.Service('check_fk', CheckKinFK, handle_check_fk)
    s3 = rospy.Service('compute_ik', ScaraKinIK, handle_compute_ik)
    s4 = rospy.Service('compute_fk', ScaraKinFK, handle_compute_fk)

    rospy.spin()


if __name__ == "__main__":
    scara_connector()
