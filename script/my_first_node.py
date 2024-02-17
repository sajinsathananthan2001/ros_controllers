#!/usr/bin/env python3

import rospy

if __name__== '__main__':
    rospy.init_node("test_node")


    rospy.loginfo("Hello from my first node")
    rospy.logwarn("Warning example from first node")
    rospy.logerr("Error example from my first node")
    rospy.sleep(1.0)
    rospy.loginfo("End of program")