#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np  # æ·»å ç¼ºå¤±çå¯¼å¥
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

path = []
with open("trajectory.txt", "r") as f:
    for line in f:
        pt = line.strip().strip("[]").split(",")
        path.append(pt)

class PID_controller:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def compute(self, now, target):
        error = target - now

        diff_error = error - self.last_error
        self.last_error = error
        
        self.integral += error
        output = self.kp * error + self.ki * self.integral + self.kd * diff_error
        
        return output


class Controller():
    def __init__(self):
        self.position_pid = PID_controller(kp=0.55, ki=0.0005, kd=0.0)  # 调整了pid参数
        rospy.init_node("pid_postion_controller", anonymous=True)

        self.idx = 0
        
        self.target_x = path[self.idx][0]  # 通过改变目标位置进行轨迹控制
        self.target_y = path[self.idx][1]
        self.x = 0.0
        self.y = 0.0

        self.pub = rospy.Publisher('/robot/control', Twist, queue_size=10)
        rospy.Subscriber('/robot/observe', LaserScan, self.observe_callback)  # 订阅当前位置，用于更新,self.cur
    def observe_callback(self, laserscan):
        if len(laserscan.ranges) < 2:
            rospy.logwarn("LaserScan æ°æ®ä¸è¶³")
            return

        r0 = laserscan.ranges[0]
        r1 = laserscan.ranges[1]

        if not np.isfinite(r0) or not np.isfinite(r1):
            rospy.logwarn("LaserScan æ°æ®ä¸­å­å¨ inf æ nan")
            return

        x = 5.0 - r0
        y = 5.0 - r1

        self.x = x        
        self.y = y

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rospy.loginfo("Current: ({:.2f}, {:.2f}), Target: ({:.2f},{:.2f})".format(self.x, self.y, self.target_x, self.target_y ))
            if (compute_distance(self.target_y, self.target_x, self.x, self.y) <= 1e-3):
                self.idx += 1
                self.target_x = path[self.idx][0]  # 通过改变目标位置进行轨迹控制
                self.target_y = path[self.idx][1]
            
            # 利用pid控制器计算位置控制量
            control_x = self.position_pid.compute(self.x, self.target_x)
            control_y = self.position_pid.compute(self.y, self.target_y)

            # 构造控制消息
            twist = Twist()
            twist.linear.x = control_x
            twist.linear.y = control_y

            self.pub.publish(twist)
            rate.sleep()  # 保证控制频率为10Hz


def compute_distance(ay, ax, by, bx):
    return math.sqrt((ax - bx)**2 + (ay - by)**2)
        
if __name__ == '__main__':
    try:
        pid_controller = Controller()
        pid_controller.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Controller terminated")
    except Exception as e:
        rospy.logerr("Error in controller: {}".format(str(e)))