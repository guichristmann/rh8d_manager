#! /usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState

rospy.init_node("spamtest", anonymous=True)

pub = rospy.Publisher("rh8d/goal_joint_state", JointState, queue_size=1)

msg = JointState()
msg.name = ['r_index_flexion', 'r_middle_flexion', 
        'r_thumb_flexion', 'r_ring_pinky_flexion']
msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    msg.position = [0.0, 0.0, 0.0, 0.0, 0.0]
    pub.publish(msg)
    rate.sleep()
    msg.position = [4.0, 4.0, 4.0, 4.0, 4.0]
    pub.publish(msg)
    rate.sleep()
