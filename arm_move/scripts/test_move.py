#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

class PoseProcessor:
    def __init__(self):
        self.previous_pose = None
        self.previous_time = None
        self.pub = rospy.Publisher('/spacenav/twist', Twist, queue_size=10)

    def pose_callback(self, data):
        current_time = rospy.Time.now()
        
        if self.previous_pose and self.previous_time:
            dt = (current_time - self.previous_time).to_sec()
            print(dt)
            if dt < 0.001:  # 避免时间差为0导致的除零错误
                dt = 0.001

            twist_msg = Twist()

            # 计算位置差值，然后计算速度
            twist_msg.linear.x = (data.position.x - self.previous_pose.position.x) / dt
            twist_msg.linear.y = (data.position.y - self.previous_pose.position.y) / dt
            twist_msg.linear.z = (data.position.z - self.previous_pose.position.z) / dt

            # 获取当前的RPY值
            roll, pitch, yaw = euler_from_quaternion([data.orientation.x,
                                                      data.orientation.y,
                                                      data.orientation.z,
                                                      data.orientation.w])

            # 获取上一时刻的RPY值
            prev_roll, prev_pitch, prev_yaw = euler_from_quaternion([self.previous_pose.orientation.x,
                                                                     self.previous_pose.orientation.y,
                                                                     self.previous_pose.orientation.z,
                                                                     self.previous_pose.orientation.w])

            # 计算RPY的速度
            twist_msg.angular.x = (roll - prev_roll) / dt
            twist_msg.angular.y = (pitch - prev_pitch) / dt
            twist_msg.angular.z = (yaw - prev_yaw) / dt

            # 发布Twist消息
            self.pub.publish(twist_msg)
            rospy.loginfo("Published twist command based on delta pose and delta time.")

        self.previous_pose = data
        self.previous_time = current_time

def main():
    rospy.init_node('delta_pose_to_twist_node', anonymous=True)
    processor = PoseProcessor()
    rospy.Subscriber("/hand_position_pose", Pose, processor.pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
