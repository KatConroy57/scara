#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import SwitchController
from scara_command.srv import SwitchControl, SwitchControlResponse


# Use controller manager to switch between velocity and position controller
def handle_switch(req):
    rospy.wait_for_service('/scara/controller_manager/switch_controller')
    
    try:
        switch_controller = rospy.ServiceProxy(
                    '/scara/controller_manager/switch_controller', SwitchController)
        
        if req.command == 'p2v': # Position -> Velocity
            ret1 = switch_controller(['joint1_velocity_controller',
                                      'joint2_velocity_controller',
                                      'joint3_velocity_controller'], 
                                     ['joint1_position_controller',
                                      'joint2_position_controller',
                                      'joint3_position_controller'], 2)
        elif req.command == 'v2p': # Velocity -> Position
            ret2 = switch_controller(['joint1_position_controller',
                                      'joint2_position_controller',
                                      'joint3_position_controller'], 
                                     ['joint1_velocity_controller',
                                      'joint2_velocity_controller',
                                      'joint3_velocity_controller'], 2)
        else:
            print 'Input should be "p2v" or "v2p"'
            return SwitchControlResponse(False)
            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return SwitchControlResponse(True)

    
def switch_control():
    rospy.init_node('switch_control_server')
    s = rospy.Service('switch_control', SwitchControl, handle_switch)

    rospy.spin()


if __name__ == "__main__":
    switch_control()

