#!/usr/bin/env python

from scara_command.srv import ScaraVelFK, ScaraVelFKResponse,\
                              ScaraVelIK, ScaraVelIKResponse
import rospy


# Handle joints velocity -> cartesian velocity
def handle_velocity_forward_kinematics(req):
    q1_dot = req.q1_dot
    q2_dot = req.q2_dot
    q3_dot = req.q3_dot
    qdot = np.array([[q1_dot],[q2_dot],[q3_dot]])
    A1 = functioncal()
    A2 = functioncall()
    A3 = functioncall()
    z0 = np.array([[0],[0],[1]])
    z1 = np.array([[0],[0],[1]])
    z2 = np.array([[0],[0],[-1]])
    o3 = np.dot(A3,np.array([[0],[0],[0],[1]]))
    np.delete(o3,3,0)
    o1 = np.dot(A1, np.array([[0], [0], [0], [1]]))
    np.delete(o1, 3, 0)
    Jv = np.column_stack((np.cross(z0,(o3-o0)),np.cross(z1,(o3-o1)),[0,0,-1]))
    Jw = np.column_stack((z0,z1,[0,0,0]))
    J = np.vstack((Jv,Jw))

    V = np.dot(J,qdot)
    Vx = V[0,0]
    Vy = V[1,0]
    Vz = V[2,0]
    Wx = V[3,0]
    Wy = V[4,0]
    Wz = V[5,0]
    
    return ScaraVelFKResponse(Vx, Vy, Vz, Wx, Wy, Wz)


# Handle cartesian velocity -> joints velocity
def handle_velocity_inverse_kinematics(req):
    Vx = req.Vx
    Vy = req.Vy
    Vz = req.Vz
    Wx = req.Wx
    Wy = req.Wy
    Wz = req.Wz
    V = np.array([[Vx],[Vy],[Vz],[Wx],[Wy],[Wz]])
    A1 = functioncall()
    A2 = functioncall()
    A3 = functioncall()
    z0 = np.array([[0], [0], [1]])
    z1 = np.array([[0], [0], [1]])
    z2 = np.array([[0], [0], [-1]])
    o3 = np.dot(A3, np.array([[0], [0], [0], [1]]))
    np.delete(o3, 3, 0)
    o1 = np.dot(A1, np.array([[0], [0], [0], [1]]))
    np.delete(o1, 3, 0)
    Jv = np.column_stack((np.cross(z0, (o3 - o0)), np.cross(z1, (o3 - o1)), [0, 0, -1]))
    Jw = np.column_stack((z0, z1, [0, 0, 0]))
    J = np.vstack((Jv, Jw))
    JxJt = np.dot(J,J.T)
    Jpinv = np.dot(np.linalg.inv(linalgJxJt),J.T)
    q = np.dot(V,Jpinv)
    q1_dot = q[0,0]
    q2_dot = q[1,0]
    q3_dot = q[2,0]

    return ScaraVelIKResponse(q1_dot, q2_dot, q3_dot)


# Server
def scara_VK_server():
    rospy.init_node('vel_kin_server')
    
    s1 = rospy.Service('vel_for_kin', ScaraVelFK, handle_velocity_forward_kinematics)
    s2 = rospy.Service('vel_inv_kin', ScaraVelIK, handle_velocity_inverse_kinematics)
    
    rospy.spin()


if __name__ == "__main__":
    scara_VK_server()
