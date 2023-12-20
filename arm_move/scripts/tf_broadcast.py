#!/usr/bin/env python3

import rospy
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import PoseStamped, TransformStamped

def callback_right_shoulder(data):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "right_shoulder"
    t.transform.translation = data.pose.position
    t.transform.rotation = data.pose.orientation

    br.sendTransform(t)

def callback_right_hand(data):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "right_hand"
    t.transform.translation = data.pose.position
    t.transform.rotation = data.pose.orientation

    br.sendTransform(t)

def main():
    rospy.init_node('pose_to_tf_broadcaster')

    rospy.Subscriber("/natnet_ros/actor01_RightShoulder/pose", PoseStamped, callback_right_shoulder)
    rospy.Subscriber("/natnet_ros/actor01_RightHand/pose", PoseStamped, callback_right_hand)

    rospy.spin()

if __name__ == '__main__':
    main()

