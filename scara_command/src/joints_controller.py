#!/usr/bin/env python

from scara_command.srv import SetJointRef, SetJointRefResponse
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties
from std_msgs.msg import Float32, Float64

import rospy
import time


# PD controller
def handle_pd_controller(req):
    # Acquire current position
    rospy.wait_for_service('gazebo/get_joint_properties')
    try:
        joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
        joint_properties = joints_properties(req.joint_name)
        q = joint_properties.position[0]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    except IndexError, e:
        print "%s"%e
     
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
def joint3_controller():
    rospy.init_node('joints_pos_controller')

    s1 = rospy.Service('set_joint_ref', SetJointRef, handle_pd_controller)

    global pub1, pub2, pub3
    pub1 = rospy.Publisher('/scara/joint1_position_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/scara/joint2_position_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
    
    rospy.spin()


if __name__ == "__main__":
    joint3_controller()
