#!/usr/bin/env python

from scara_command.srv import ScaraVelFK, ScaraVelFKResponse,\
                              ScaraVelIK, ScaraVelIKResponse
import rospy


# Handle joints velocity -> cartesian velocity
def handle_velocity_forward_kinematics(req):
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3
    q1_dot = req.q1_dot
    q2_dot = req.q2_dot
    q3_dot = req.q3_dot

    Vx = 1
    Vy = 1
    Vz = 0
    Wx = 0
    Wy = 0
    Wz = 0
    
    return ScaraVelFKResponse(Vx, Vy, Vz, Wx, Wy, Wz)


# Handle cartesian velocity -> joints velocity
def handle_velocity_inverse_kinematics(req):
    q1 = req.q1
    q2 = req.q2
    q3 = req.q3
    Vx = req.Vx
    Vy = req.Vy
    Vz = req.Vz
    Wx = req.Wx
    Wy = req.Wy
    Wz = req.Wz

    q1_dot = 0
    q2_dot = 0
    q3_dot = 0

    return ScaraVelIKResponse(q1_dot, q2_dot, q3_dot)


# Server
def scara_VK_server():
    rospy.init_node('vel_kin_server')
    
    s1 = rospy.Service('vel_for_kin', ScaraVelFK, handle_velocity_forward_kinematics)
    s2 = rospy.Service('vel_inv_kin', ScaraVelIK, handle_velocity_inverse_kinematics)
    
    rospy.spin()


if __name__ == "__main__":
    scara_VK_server()
