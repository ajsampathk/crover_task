#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

class KFpose:


    def __init__(self):
        rospy.init_node("KFPose")
        rospy.Subscriber("/sensors/gnss/odom",Odometry,self.gnss_callback)
        rospy.Subscriber("/sensors/odom",Odometry,self.sensor_odom_callback)
        self.pose_publisher = rospy.Publisher("/car/pose",PoseStamped,queue_size=10)
        self.odom_prev_ts=None
        self.dt=0
        self.predict=False
        self.update=False
        self.updated=False
        self.first_message=True
        self.state_estimate_k=None
        self.state_estimate_k_minus_1=None
        self.state_estimate_k_pub=None


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

        return B


    def predict_model(self):
        # predict state
        if self.predict:
            self.state_estimate_k = self.A_k_minus_1.dot(self.state_estimate_k_minus_1)
            # rospy.loginfo("State Predict:{}".format(self.state_estimate_k))
        # predict state covariance P
            self.P_k = self.A_k_minus_1.dot(self.P_k_minus_1).dot(self.A_k_minus_1.T) + (self.Q_k)
            self.predict=False

    def update_model(self):
        if self.update:
            measurement_residual_y_k = self.z_k_observation_vector - (
            (self.H_k.dot(self.state_estimate_k)))
            S_k = self.H_k.dot(self.P_k).dot(self.H_k.T)
            K_k = self.P_k.dot(self.H_k.T).dot(np.linalg.pinv(S_k))
            self.state_estimate_k_pub = self.state_estimate_k + (K_k.dot(measurement_residual_y_k))
            rospy.loginfo("State Predict(update):{}".format(self.state_estimate_k_pub))

            self.P_k = self.P_k - (K_k.dot(self.H_k).dot(self.P_k))
            self.update=False
            self.updated=True

    def publish(self):
        if self.updated:
            pose=PoseStamped()
            pose.pose.position.x = self.state_estimate_k_pub[0,0]
            pose.pose.position.y = self.state_estimate_k_pub[1,0]

            quaternion = quaternion_from_euler(0,0,self.state_estimate_k_pub[2,0])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id="map"

            self.pose_publisher.publish(pose)
            self.updated=False


    def gnss_callback(self,msg):
        self.z_k_observation_vector = np.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[self.state_estimate_k[2]]])
        self.update=True


    def sensor_odom_callback(self,msg):

        if not self.first_message:

            self.dt = (msg.header.stamp - self.odom_prev_ts).to_sec()
            # rospy.loginfo("dt:{}".format(self.dt))
        else:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            self.state_estimate_k_minus_1 = np.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[yaw]])
            self.first_message=False


        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.state_estimate_k_minus_1 = np.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[yaw]])

        cov = msg.pose.covariance
        self.Q_k = np.array([[cov[0],cov[1],cov[5]],
                                [cov[6],cov[7],cov[11]],
                                    [cov[30],cov[31],cov[35]]])

        self.predict= not self.first_message
        self.odom_prev_ts=msg.header.stamp


if __name__ == '__main__':
    pose = KFpose()
    while not rospy.is_shutdown():
        pose.predict_model()
        pose.update_model()
        pose.publish()