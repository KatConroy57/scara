import rospy
import numpy
import math

# Import gazebo_msgs, std_msgs
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from gazebo_msgs.srv import GetJointProperties, GetLinkState


# Acquire gazebo joint variables
def acquire_joints():
    rospy.wait_for_service('gazebo/get_joint_properties')
    try:
        joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
        joint1_properties = joints_properties("joint1")
        q1 = joint1_properties.position[0]
        joint2_properties = joints_properties("joint2")
        q2 = joint2_properties.position[0]
        joint3_properties = joints_properties("joint3")
        q3 = joint3_properties.position[0]

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return q1, q2, q3


# Acquire gazebo end of effector coordinate
def acquire_coordinates():
    rospy.wait_for_service('gazebo/get_link_state')
    try:
        coordinates_gene = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        coordinates = coordinates_gene("link3", "world")
	# Position
	x = coordinates.link_state.pose.position.x
	y = coordinates.link_state.pose.position.y
	z = coordinates.link_state.pose.position.z
	# Orientation (Quaternion)
	qt1 = coordinates.link_state.pose.orientation.x
	qt2 = coordinates.link_state.pose.orientation.y
	qt3 = coordinates.link_state.pose.orientation.z
	qt0 = coordinates.link_state.pose.orientation.w
	# convert to radian
	phi = math.atan2(2*(qt0*qt1 + qt2*qt3), 1-2*(qt1*qt1 + qt2*qt2))
	theta = math.asin(2*(qt0*qt2 - qt3*qt1))
	psi = math.atan2(2*(qt0*qt3 + qt1*qt2), 1-2*(qt2*qt2 + qt3*qt3))
	
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return x, y, z, phi, theta, psi


# Convert float32multiarray to numpy array 
def ma2np(ma):
    arr_list = []
    arr_row = []
    count = 0

    for element in ma.data:
        arr_row.extend([element])
        count += 1
        if count == ma.layout.dim[1].size:
            arr_list.append(arr_row)
            arr_row = []
            count = 0

    arr = numpy.asarray(arr_list)
    return arr


# Convert numpy array to float32multiarray
def np2ma(arr):
    ma = Float32MultiArray()

    content = []
    for row in arr:
        content.extend(row)
    ma.data = content

    ma.layout.dim = [MultiArrayDimension(label="row", size=len(arr)),\
                     MultiArrayDimension(label="column", size=len(arr[0]))]

    return ma