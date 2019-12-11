#!/usr/bin/env python

from scara_command.srv import SetJointRef, SetJointRefResponse,\
                              SetCartesianPos, SetCartesianPosResponse,\
                              ScaraKinIK
from std_msgs.msg import Float32, Float64

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
    rospy.wait_for_service('inv_kin')
    try:
        inv_kinematic = rospy.ServiceProxy('inv_kin', ScaraKinIK)
        res = inv_kinematic(req.x, req.y, req.z, 0, 0, 0)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    if res.success:
        # Apply joint effort using controller
        pub1.publish(res.q1)
        pub2.publish(res.q2)
        pub3.publish(res.q3)
        return SetCartesianPosResponse(True)
    
    print "IK failed, the coordinate provided may be invalid."
    return SetCartesianPosResponse(False)


# Connector services
def joints_pos_controller():
    rospy.init_node('joints_pos_controller')

    s1 = rospy.Service('set_joint_pos_ref', SetJointRef, handle_joint_ref)
    s2 = rospy.Service('set_cartesian_pos_ref', SetCartesianPos, handle_cartesian_ref)

    global pub1, pub2, pub3
    pub1 = rospy.Publisher('/scara/joint1_position_controller/command', Float64, queue_size=1)
    pub2 = rospy.Publisher('/scara/joint2_position_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)
    
    rospy.spin()


if __name__ == "__main__":
    joints_pos_controller()