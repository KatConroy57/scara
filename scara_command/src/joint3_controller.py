#!/usr/bin/env python

from scara_command.srv import SetJointRef, SetJointRefResponse
from gazebo_msgs.srv import ApplyJointEffort, GetJointProperties
from std_msgs.msg import Float32, Float64

import rospy
import time


# PD controller
def handle_pd_controller(reference):
    # Acquire current position
    rospy.wait_for_service('gazebo/get_joint_properties')
    try:
        joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
	joint3_properties = joints_properties("joint3")
	q = joint3_properties.position[0]
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
     
    # Publish only once to gazebo PID controller
    pub3.publish(Float64(reference.ref))

    return SetJointRefResponse(True)
    
    ''' Original Defined Controller
    # Control Time
    Time = 3
    print "controlling joint 3 for ", Time, " seconds"
    start_time = time.time()
    curr_time = start_time
    pre_e = reference.ref - q # for calculating e_dot

    # Start controlling
    while (time.time()-start_time < Time):
        # Error
        sampling_time = time.time() - curr_time
        curr_time = time.time()

        e = reference.ref - q
        e_dot = (e-pre_e)/sampling_time
        pre_e = e

        # PD Controller
        kp = 3
        kd = 0.2
        u = kp * e + kd * e_dot

        # Send back the joint effort
        rospy.wait_for_service('gazebo/apply_joint_effort')
        try:
	    apply_effort = rospy.ServiceProxy('gazebo/apply_joint_effort', ApplyJointEffort)
	    status = apply_effort("joint3", u, rospy.Time(0, 0), rospy.Duration(10, 0))
	
	    if not status.success:
	        print status.status_message
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    return SetJointRefResponse(True)
    '''


# Connector services
def joint3_controller():
    rospy.init_node('joint3_pos_controller')

    global pub3
    pub3 = rospy.Publisher('/scara/joint3_position_controller/command', Float64, queue_size=1)

    s3 = rospy.Service('set_joint3_ref', SetJointRef, handle_pd_controller)
    

    rospy.spin()


if __name__ == "__main__":
    joint3_controller()
