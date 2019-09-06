#! /usr/bin/env python2

import sys
from os import path
from sensor_msgs.msg import JointState
import rospy
import rospkg
import yaml
import rosparam
import pydynamixel as pd

# Get path of this package
rospack = rospkg.RosPack()
pkg_path = rospack.get_path("rh8d_manager")

rospy.init_node("RH8D1_Manager")

# -------- SETUP ---------
# Load parameters from file
params = yaml.load(open(path.join(pkg_path, "config/params.yaml"), 'r'))

port_path = params['port']['port']
protocol_ver = params['port']['protocol']
baudrate = params['port']['baudrate']
joint_ids = params['IDs']

# Set default control table for the PyDynamixel library
pd.setDefaultCtrlTable("RH8D-1")

# Create port object
port = pd.DxlComm(port_path, protocol_version=protocol_ver, baudrate=baudrate)

# Create dictionary with name for key and joint object for value
joints = {k: pd.Joint(v) for k, v in joint_ids.items()}

# Attach joints to port
port.attach_joints([joint for joint in joints.values()])
# ---------------------

def goal_joint_state_callback(msg):
    global flag_sync_write

    joint_names = msg.name
    joint_pos = msg.position

    for i, jn in enumerate(joint_names):
        joints[jn].set_goal_value(joint_pos[i])

    # Signals to the main loop to call sync write after bulk reading
    flag_sync_write = True

joint_state_pub = rospy.Publisher("/rh8d/present_joint_states", JointState, 
        queue_size=1)
joint_state_sub = rospy.Subscriber("/rh8d/goal_joint_state", JointState,
        goal_joint_state_callback)

rate = rospy.Rate(60)
flag_sync_write = False
while not rospy.is_shutdown():
    # 1. Bulk read all motors attached to the port
    try:
        all_joints_pos = port.bulk_read_present_position()

        joint_state_msg = JointState()
        joint_state_msg.name = list(joints.keys())
        joint_state_msg.position = all_joints_pos

        joint_state_pub.publish(joint_state_msg)

    except RuntimeError as e:
        rospy.loginfo("Bulk read fail: {}".format(e))

    # 2. If the .goal_values have been set call sync write
    if flag_sync_write:
        port.sync_write_goal_position()
        flag_sync_write = False

    rate.sleep()

rospy.spin()
