#!/usr/bin/env python

from scara_command.srv import SetJointRef, SetJointRefResponse,\
                              SetCartesianVel, SetCartesianVelResponse,\
                              ScaraVelIK
from std_msgs.msg import Float32, Float64
from helper import acquire_joints

import rospy
import time


def handle_joint_ref(req):
    # Publish once with gazebo PID controller
    if req.joint_name == 'joint1':
        pub1.publish(req.ref)
        return SetJointRefResponse(True)
    elif req.joint_name == 'joint2':
        pub2.publish(req.ref)
        return SetJointRefResponse(True)
    elif req.joint_name == 'joint3':
        pub3.publish(req.ref)
        return SetJointRefResponse(True)

    else:
        rospy.loginfo("Input Joint Name Invalid")
        return SetJointRefResponse(False)


def handle_cartesian_ref(req):
    # Perform inverse kinematics
    rospy.wait_for_service('vel_inv_kin')
    flag = True

    timeout = time.time() + 3   # Set 3 seconds
    while flag:
        joints = acquire_joints()
        try:
            vel_inv_kinematic = rospy.ServiceProxy('vel_inv_kin', ScaraVelIK)
            res = vel_inv_kinematic(joints[0], joints[1], joints[2],\
                                    req.Vx, req.Vy, req.Vz, req.Wx,\
                                    req.Wy, req.Wz)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if res.success:
            # Apply joint effort using controller
            pub1.publish(res.q1_dot)
            pub2.publish(res.q2_dot)
            pub3.publish(res.q3_dot)
        else:
            flag = False

        if time.time() > timeout:
            break
    
    if flag: # If succeed
        print "Velocity controllers work for 3 seconds"
    else: # If failed
        print "Velocity IK failed, the velocity provided may be invalid."

    # Stop controlling
    pub1.publish(0)
    pub2.publish(0)
    pub3.publish(0)
    return SetCartesianVelResponse(flag)
    

# Connector services
def joints_vel_controller():
    rospy.init_node('joints_vel_controller')

    s1 = rospy.Service('set_joint_vel_ref', SetJointRef, handle_joint_ref)
    s2 = rospy.Service('set_cartesian_vel_ref', SetCartesianVel, handle_cartesian_ref)

    global pub1, pub2, pub3
    pub1 = rospy.Publisher('/scara/joint1_velocity_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/scara/joint2_velocity_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/scara/joint3_velocity_controller/command', Float64, queue_size=1)
    
    rospy.spin()


if __name__ == "__main__":
    joints_vel_controller()