#!/usr/bin/env python

from scara_command.srv import ScaraVelFK, ScaraVelFKResponse, ScaraHomoMatrix, ScaraHomoMatrixResponse, ScaraVelIK, ScaraVelIKResponse
import rospy
import numpy as np
import rosservice
from helper import ma2np



# Handle joints velocity -> cartesian velocity..
def handle_velocity_forward_kinematics(req):
    print "Received Forward Request"
    q1_dot = req.q1_dot
    q2_dot = req.q2_dot
    q3_dot = req.q3_dot
    qdot = np.array([[q1_dot],[q2_dot],[q3_dot]])
    #[q1,q2,q3] = acquire_joints()..
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3

    #Now to find A1 A2 A3.
    rospy.wait_for_service('homogeneous_matrix')
    try:
        findA = rospy.ServiceProxy('homogeneous_matrix',ScaraHomoMatrix)
        findAs = findA(q1,q2,q3)
        A1 = ma2np(findAs.A1)
        A2 = ma2np(findAs.A2)
        A3 = ma2np(findAs.A3)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    z0 = np.array([[0],[0],[1]])
    z1 = np.array([[0],[0],[1]])
    z2 = np.array([[0],[0],[-1]])
    o3 = np.dot(A3,np.array([[0],[0],[0],[1]]))
    o3 = np.delete(o3,3,0)
    print o3
    o1 = np.dot(A1, np.array([[0], [0], [0], [1]]))
    o1 = np.delete(o1, 3, 0)
    print o1
    Jv = np.column_stack((np.cross(z0.T,o3.T).T,np.cross(z1.T,(o3.T-o1.T)).T,[[0],[0],[-1]]))
    Jw = np.column_stack((z0,z1,[0,0,0]))
    J = np.vstack((Jv,Jw))

    V = np.dot(J,qdot)
    Vx = V[0,0]
    Vy = V[1,0]
    Vz = V[2,0]
    Wx = V[3,0]
    Wy = V[4,0]
    Wz = V[5,0]

    print "Completed Forward Kin"
    return ScaraVelFKResponse(Vx, Vy, Vz, Wx, Wy, Wz)


# Handle cartesian velocity -> joints velocity
def handle_velocity_inverse_kinematics(req):
    print "Received Inverse Request"
    Vx = req.Vx
    Vy = req.Vy
    Vz = req.Vz
    Wx = req.Wx
    Wy = req.Wy
    Wz = req.Wz
    V = np.array([[Vx],[Vy],[Vz]])
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3

    # To find A1 A2 A3
    rospy.wait_for_service('homogeneous_matrix')
    try:
        findA = rospy.ServiceProxy('homogeneous_matrix', ScaraHomoMatrix)
        findAs = findA(q1, q2, q3)
        A1 = ma2np(findAs.A1)
        A2 = ma2np(findAs.A2)
        A3 = ma2np(findAs.A3)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    A13 = np.dot(A1,np.dot(A2,A3))
    z0 = np.array([[0], [0], [1]])
    z1 = np.array([[0], [0], [1]])
    z2 = np.array([[0], [0], [-1]])
    o3 = np.dot(A13, np.array([[0], [0], [0], [1]]))
    o3 = np.delete(o3, 3, 0)
    o1 = np.dot(A1, np.array([[0], [0], [0], [1]]))
    o1 = np.delete(o1, 3, 0)
    Jv = np.column_stack((np.cross(z0.T, (o3).T).T, np.cross(z1.T, (o3.T - o1.T)).T, [[0], [0], [-1]]))
    Jw = np.column_stack((z0, z1, [0, 0, 0]))
    J = np.vstack((Jv, Jw))
    print Jv
    #JxJt = np.dot(J,J.T)..
    #Jpinv = np.dot(np.linalg.inv(JxJt),J.T)
    Jinv = np.linalg.inv(Jv)
    q = np.dot(Jinv,V)
    print q
    q1_dot = q[0,0]
    q2_dot = q[1,0]
    q3_dot = q[2,0]


    print "Completed Inverse Kin"
    return ScaraVelIKResponse(q1_dot, q2_dot, q3_dot)


# Server
def scara_VK_server():
    rospy.init_node('vel_kin_server')
    s1 = rospy.Service('vel_for_kin', ScaraVelFK, handle_velocity_forward_kinematics)
    s2 = rospy.Service('vel_inv_kin', ScaraVelIK, handle_velocity_inverse_kinematics)
    print "Ready for Vel Kin 7"
    
    rospy.spin()


if __name__ == "__main__":
    scara_VK_server()
