#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose

def pose_callback(msg):

    #rospy.loginfo(msg)
    obj = Pose()
    obj = msg
    rospy.loginfo(obj.x)

if __name__=="__main__":
    rospy.init_node("fst_subscriber")
    rospy.loginfo("Subscribing Started")

    sub = rospy.Subscriber("/turtle1/pose",Pose,callback=pose_callback)

    rospy.spin()