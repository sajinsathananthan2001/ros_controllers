#!/usr/bin/env python3

import rospy

if __name__=='__main__':

    rospy.init_node("test_node2")

    rospy.loginfo("Second test Node started")

    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello World from node 2")
        rate.sleep()