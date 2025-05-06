#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
基于ROS的二维位置PID控制器
功能：控制机器人沿预定路径移动，并在到达点时放置标记
"""

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState

# 全局路径点容器
path = []

# 路径文件读取（适配Python 2/3）
try:
    with open("/home/headless/catkin_ws/src/cylinder_robot/scripts/trajectory.txt", "r") as f:
        for line in f:
            line = line.strip()  # 去除首尾空白字符
            if not line:  # 跳过空行
                continue
            
            # 解析格式示例：[1.0, 2.0] -> ['1.0', ' 2.0']
            pt = line.strip("[]").split(",")
            
            # 验证数据有效性并转换类型
            if len(pt) >= 2:
                path.append([
                    float(pt[0].strip()),  # x坐标
                    float(pt[1].strip())   # y坐标
                ])
except IOError as e:
    rospy.logerr("路径文件读取失败: {}".format(str(e)))
    exit(1)

def create_marker(point, marker_id):
    """创建可视化标记
    Args:
        point: [x,y,z] 坐标位置
        marker_id: 标记唯一ID（避免重复覆盖）
    Returns:
        visualization_msgs/Marker 对象
    """
    marker = Marker()
    marker.header.frame_id = "map"       # 使用Gazebo世界坐标系
    marker.type = Marker.SPHERE          # 球形标记
    marker.action = Marker.ADD
    marker.scale.x = 0.1                # 标记尺寸(米)
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0                # 不透明度
    marker.color.r = 1.0                # RGB红色
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.0        # 地面高度
    marker.id = marker_id               # 唯一标识
    marker.lifetime = rospy.Duration(0) # 永久显示
    return marker

class PID_controller:
    """离散PID控制器实现"""
    def __init__(self, kp, ki, kd):
        """
        Args:
            kp: 比例增益
            ki: 积分增益 
            kd: 微分增益
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0  # 上一次误差
        self.integral = 0.0    # 误差积分项

    def compute(self, current, target):
        """计算控制量
        Args:
            current: 当前状态值
            target: 目标值
        Returns:
            控制量输出
        """
        error = target - current
        
        # 微分项计算
        diff_error = error - self.last_error
        self.last_error = error
        
        # 积分项累加
        self.integral += error
        
        # PID公式
        return self.kp * error + self.ki * self.integral + self.kd * diff_error

class Controller():
    def __init__(self):
        # 初始化XY轴PID控制器
        self.x_pid = PID_controller(kp=5.75, ki=0.0005, kd=0.0)
        self.y_pid = PID_controller(kp=5.65, ki=0.0001, kd=0.0)
        
        # ROS节点初始化
        rospy.init_node("pid_position_controller", anonymous=True)
        

        self.idx = 0                      # 当前路径点索引
        self.target_x, self.target_y = path[self.idx]  # 目标坐标
        self.x = self.y = 0.0             # 当前估计位置
        
        # ROS通信接口
        self.pub = rospy.Publisher('/robot/control', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/path_markers', Marker, queue_size=10)
        # rospy.Subscriber('/robot/observe', LaserScan, self.observe_callback)

        self.model_state_sub = rospy.Subscriber(
            "/robot/esti_model_state", 
            ModelState, 
            self.model_state_callback,
            queue_size=10
        )
    
    def model_state_callback(self, msg):
        """处理接收到的模型状态信息"""
        self.esti_model_state = msg
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def observe_callback(self, laserscan):
        """激光雷达数据回调函数
        Args:
            laserscan: sensor_msgs/LaserScan 消息
        """
        # 数据有效性检查
        if len(laserscan.ranges) < 2:
            rospy.logwarn("无效的激光数据：采样点不足")
            return
        
        r0, r1 = laserscan.ranges[0], laserscan.ranges[1]
        

        self.x = 5 - r0
        self.y = 5 - r1


    def run(self):
        """主控制循环"""
        rate = rospy.Rate(10)  # 10Hz控制频率
        
        while not rospy.is_shutdown():
            # 状态监控日志
            rospy.loginfo("当前位置: ({:.4f}, {:.4f}) → 目标点: ({:.4f}, {:.4f})".format(
                self.x, self.y, self.target_x, self.target_y))
            
            # 计算到目标点距离
            distance = math.sqrt((self.target_x - self.x)**2 + 
                               (self.target_y - self.y)**2)
            rospy.loginfo("距离目标: {:.4f} 米".format(distance))
            
            # 到达判定（0.1米阈值）
            if distance <= 0.1:
                # 发布位置标记
                marker = create_marker((self.x, self.y, 0.0), self.idx)
                self.marker_pub.publish(marker)
                rospy.loginfo("在 ({:.4f}, {:.4f}) 放置标记#{}".format(
                    self.x, self.y, self.idx))
                
                # 更新到下一个路径点（防越界）
                self.idx = self.idx + 1
                if self.idx == len(path) - 1: return 
                self.target_x, self.target_y = path[self.idx]
                rospy.loginfo("切换至路径点#{}".format(self.idx))
            
            # 生成控制指令
            twist = Twist()
            twist.linear.x = self.x_pid.compute(self.x, self.target_x)  # X轴控制
            twist.linear.y = self.y_pid.compute(self.y, self.target_y)  # Y轴控制
            self.pub.publish(twist)
            
            rate.sleep()


def compute_distance(ax, ay, bx, by):
    return math.sqrt((ax - bx)**2 + (ay - by)**2)
        
if __name__ == '__main__':
    try:
        pid_controller = Controller()
        pid_controller.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Controller terminated")
    except Exception as e:
        rospy.logerr("Error in controller: {}".format(str(e)))