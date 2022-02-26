#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class KFpose:


    def __init__(self):
        rospy.init_node("KFPose")
        rospy.Subscriber("/sensors/gnss/odom",Odometry,self.gnss_callback)
        rospy.Subscriber("/sensors/odom",Odometry,self.sensor_odom_callback)

        self.odom_prev_ts=None
        self.dt=0
        self.u = None
        self.predict=False
        self.update=False
        self.first_message=True
        self.state_estimate_k=None
        self.state_estimate_k_minus_1=None

        self.P_k=None
        self.P_k_minus_1 = np.array([[0.1,0,0],
                                        [0,0.1,0.1],
                                            [0,0,0.1]])

        self.A_k_minus_1 = np.array([[1.0,  0,   0],
                                        [ 0 , 1.0,   0],
                                            [  0,  0, 1.0]])
        
        self.Q_k=np.array([[1.0,   0,   0],
                                [  0, 1.0,   0],
                                    [  0,   0, 1.0]])

        self.H_k = np.array([[1.0,  0,   0],
                                    [  0, 1.0,   0],
                                    [  0,  0, 1.0]])

        self.R_k = np.array([[1.0,   0,    0],
                                    [  0, 1.0,    0],
                                    [  0,    0, 1.0]])  



    def getB(self):
        B = np.array([  [np.cos(self.state_estimate_k_minus_1[2])*self.dt, 0],
                                    [np.sin(self.state_estimate_k_minus_1[2])*self.dt, 0],
                                    [0, self.dt]])
        return B


    def predict_model(self):
        # predict state
        if self.predict:
            self.state_estimate_k = self.A_k_minus_1.dot(self.state_estimate_k_minus_1) + (self.getB()).dot(self.u)
            rospy.loginfo("State Predict:{}".format(self.state_estimate_k))
        # predict state covariance P
            self.P_k = self.A_k_minus_1.dot(self.P_k_minus_1).dot(self.A_k_minus_1.T) + (self.Q_k)
            self.predict=False

    def update_model(self):
        if self.update:
            measurement_residual_y_k = self.z_k_observation_vector - (
            (self.H_k.dot(self.state_estimate_k)))
            S_k = self.H_k.dot(self.P_k).dot(self.H_k.T) + self.R_k
            K_k = self.P_k.dot(self.H_k.T).dot(np.linalg.pinv(S_k))
            self.state_estimate_k = self.state_estimate_k + (K_k.dot(measurement_residual_y_k))
            rospy.loginfo("State Predict(update):{}".format(self.state_estimate_k))

            self.P_k = self.P_k - (K_k.dot(self.H_k).dot(self.P_k))
            self.update=False

    def gnss_callback(self,msg):
        self.z_k_observation_vector = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,self.state_estimate_k[2]])
        self.update=True


    def sensor_odom_callback(self,msg):

        if not self.first_message:

            self.dt = msg.header.stamp - self.odom_prev_ts
        else:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            self.state_estimate_k_minus_1 = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,yaw])

        #update matrix u
        self.u = np.array([[msg.twist.twist.linear.x],
                            [msg.twist.twist.angular.z]])
        cov = msg.twist.covariance
        self.Q_k = np.array([[cov[0][0],cov[0][1],cov[0][5]],
                                [cov[1][0],cov[1][1],cov[1][5]],
                                    [cov[5][0],cov[5][1],cov[5][5]]])

        self.predict= not self.first_message
        self.odom_prev_ts=msg.header.stamp


if __name__ == '__main__':
    pose = KFpose()
    while not rospy.is_shutdown():
        pose.predict_model()
        pose.update_model()