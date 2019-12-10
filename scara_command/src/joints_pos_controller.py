#!/usr/bin/env python

from scara_command.srv import SetJointRef, SetJointRefResponse
from std_msgs.msg import Float32, Float64

import rospy
import time


# PD controller
def handle_pd_controller(req):
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


# Connector services
def joints_pos_controller():
    rospy.init_node('joints_pos_controller')

    s1 = rospy.Service('set_joint_pos_ref', SetJointRef, handle_pd_controller)

    global pub1, pub2, pub3
    pub1 = rospy.Publisher('/scara/joint1_position_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/scara/joint2_position_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
    
    rospy.spin()


if __name__ == "__main__":
    joints_pos_controller()