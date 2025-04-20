#!/usr/bin/env python
# -*- coding: utf-8 -*-
# from this import d
import rospy
import random
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from scipy import linalg as lnr
 
class Driver(object):
    # constructor
    def __init__(self):
        self.time_save = 0
        self.name = 'cylinderRobot'

        ## ===================================== Edit bellow =====================================
        ## ==========================================================================   vvvvvvvvvv

        # Define publisher object.  Publish the state of robot to topic "/gazebo/set_model_state"
        #   message type is ModelState
        self.pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        # Define subscriber object. Subscrip the signal from  topic  "/robot/control" sent by teleop
        #   message type is Twist
        #   set the callback function as self.callback_control
        rospy.Subscriber("/robot/control", Twist, self.callback_control)

        # member variable saving system state.
        self.state = np.zeros([4,1])    
        # state = [ px; vx; py; vy ]
        #   dx/dt = Ax(t) + Bu(t) + w(t), 
        #   cov[w(t), w(t)] = Sigma_w

        # Define matrix of continuous system:  A, B (by numpy). Assume the mass of robot  m = 10
        self.mass = 10        
        self.A = np.array([
            [0,0,0,0],
            [1,0,0,0],
            [0,0,0,0],
            [0,1,0,0]
        ])
        self.B = np.array([
            [1 / self.mass, 0],
            [0,0],
            [0, 1 / self.mass],
            [0, 0]
        ])
        self.Sigma_w = np.eye(4)*0.00001

        ## ==========================================================================  ^^^^^^^^^^
        ## ===================================== Edit above =====================================
        self.n, self.m = self.B.shape

    # callback function of subscriber
    def callback_control(self, twist):
        if self.time_save == 0:
            self.time_save = rospy.get_time()
        else:
            # measure the dt since the last sampling time
            dt = rospy.get_time() - self.time_save
            self.time_save = rospy.get_time()

            # get the control signal:  u[0] = f_x, u[1] = f_y
            u = np.zeros([2,1])
            u[0] = twist.linear.x
            u[1] = twist.linear.y #这里改成了Linear.y，不是旋转！

            # forward dynamics
            self.state = self.forward_dynamics(u, dt)
            # generate and send the robot state to Gazebo
            self.sendStateMsg()

    # forward dynamics of robot. 
    def forward_dynamics(self, u, dt):

        # discretize the continuous system to discrete.
        Atilde, Btilde, Sigma_w_tilde = self._discretization_Func(dt)

        # generate the Gaussian noise
        w = np.random.multivariate_normal(np.zeros([self.n]), Sigma_w_tilde).reshape([self.n, 1])

        x = Atilde.dot(self.state) + Btilde.dot(u) + w
        return x



    # discretization function
    def _discretization_Func(self, dt):

        ## ===================================== Edit bellow =====================================
        ## ==========================================================================   vvvvvvvvvv


        # Please implementation the discretization function here
        Atilde = np.array([
            [1, 0,  0,  0],
            [dt,1,  0,  0],
            [0, 0,  1,  0],
            [0, 0,  dt, 1]
        ])
        Btilde = np.array([
            [dt/self.mass,      0],
            [dt*dt/(2*self.mass), 0],
            [0,      dt/self.mass],
            [0, dt*dt/(2*self.mass)]
        ])
        q1 = self.Sigma_w[0,0]
        q2 = self.Sigma_w[1,1]
        q3 = self.Sigma_w[2,2]
        q4 = self.Sigma_w[3,3]
        Sigma_w_tilde = np.array([
            [dt*q1,         dt*dt/2*q1,                 0,          0],
            [dt*dt/2*q1,    (dt*q2)+(dt*dt*dt/3*q1),    0,          0],
            [0,             0,                          dt*q3,      dt*dt/2*q3],
            [0,             0,                          dt*dt/2*q3, (dt*q4)+(dt*dt*dt/3*q3)],
        ])


        return Atilde, Btilde, Sigma_w_tilde

        ## ==========================================================================  ^^^^^^^^^^
        ## ===================================== Edit above =====================================



    def sendStateMsg(self):
        msg = ModelState()
        msg.model_name = self.name
        msg.pose.position.x = self.state[0]
        msg.pose.position.y = self.state[2]
        self.pub.publish(msg)
 
       
       
 
if __name__ == '__main__':
    try:
        rospy.init_node('driver', anonymous=True)
        driver = Driver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
