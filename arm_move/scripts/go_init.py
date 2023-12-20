#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

def send_target_pose():
    # 初始化节点
    rospy.init_node('pose_publisher_node', anonymous=True)

    # 创建publisher
    pub = rospy.Publisher('my_cartesian_motion_controller/target_frame', PoseStamped, queue_size=10)

    # 构建PoseStamped消息
    target_pose = PoseStamped()

    # 设置header信息
    target_pose.header.seq = 4472973
    target_pose.header.stamp.secs = 4506
    target_pose.header.stamp.nsecs = 537000000
    target_pose.header.frame_id = "base_link"

    # 设置position信息
    target_pose.pose.position.x = -0.133658481784
    target_pose.pose.position.y = -0.120752090175
    target_pose.pose.position.z = 0.274240441988

    # 设置orientation信息
    target_pose.pose.orientation.x = -0.1936938099
    target_pose.pose.orientation.y = 0.466732967199
    target_pose.pose.orientation.z = -0.242958229725
    target_pose.pose.orientation.w = 0.828018454834

    # 等待连接到publisher
    rospy.sleep(1)

    # 发布消息
    pub.publish(target_pose)
    rospy.loginfo("Target pose sent!")

if __name__ == '__main__':
    try:
        send_target_pose()
    except rospy.ROSInterruptException:
        pass
