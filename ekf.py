#!/usr/bin/env python3

# Extended Kalman Filter

import math
from re import T
import numpy
import time

import rclpy
from rclpy.node import Node

from state_estimator_msgs.msg import RobotPose
from state_estimator_msgs.msg import SensorData

class Estimator(Node):
    def __init__(self):
        super().__init__('estimator')

        # Publisher to publish state estimate
        self.pub_est = self.create_publisher(RobotPose, "/robot_pose_estimate", 1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.05

        # Subscribe to command input and sensory output of robot
        self.sensor_sub = self.create_subscription(SensorData, "/sensor_data", self.sensor_callback, 1)
        
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------

    def estimate(self, sens: SensorData):
        '''This function gets called every time the robot publishes its control 
        input and sensory output. You must make use of what you know about 
        extended Kalman filters to come up with an estimate of the current
        state of the robot and covariance matrix. The SensorData message 
        contains fields 'vel_trans' and 'vel_ang' for the commanded 
        translational and rotational velocity respectively. Furthermore, 
        it contains a list 'readings' of the landmarks the robot can currently
        observe

        Args:
            sens: incoming sensor message
        '''
        # Implement the dynamic model to predict the state
        x_k = self.x[0, 0]
        y_k = self.x[1, 0]
        theta_k = self.x[2, 0]

        vel_trans = sens.vel_trans
        vel_ang = sens.vel_ang
        t = self.step_size

        # Update state prediction using the dynamic model
        x_pred = x_k + t * vel_trans * math.cos(theta_k)
        y_pred = y_k + t * vel_trans * math.sin(theta_k)
        theta_pred = theta_k + self.step_size * vel_ang
        
        # Update self.x
        self.x[0, 0] = x_k + t * vel_trans * math.cos(theta_k)
        self.x[1, 0] = y_k + t * vel_trans * math.sin(theta_k)
        self.x[2, 0] = theta_k + self.step_size * vel_ang

        # Compute the state transition matrix (F) & update the prediction using the dynamic model and white noise
        F = self.compute_state_transition_matrix(theta_k, vel_trans, t)
        self.P = numpy.dot(numpy.dot(F, self.P), F.T) + self.V

        # Update Step (Correction) for each observed landmark
        for reading in sens.readings:
            if reading.range>0.1:
                noise_range = 0.1
                noise_bearing = 0.05
                landmark_range = reading.range
                landmark_bearing = reading.bearing
                landmark = reading.landmark
                landmark_x = landmark.x
                landmark_y = landmark.y

                # Compute sensor white noise matrix (W)
                W_k = numpy.zeros((2,2))
                W_k[0,0] = noise_range
                W_k[1,1] = noise_bearing
                W_k_1 = W_k

                # Compute the measurement matrix (H) linearized around the predicted state (x_predicted(k+1)), sensor range and bearing
                H, sensor_range, sensor_bearing = self.compute_measurement_matrix_range_bearing(landmark_x, landmark_y, x_pred, y_pred, theta_pred)

                # Compute matrix (S)
                S = numpy.dot(numpy.dot(H,self.P),H.T) + W_k_1

                # Compute matix (R)
                R = numpy.dot(numpy.dot(self.P,H.T), numpy.linalg.inv(S))

                # Compute innovation matrix (Innov)
                Innov = numpy.zeros((2,1))
                Innov[0,0] = landmark_range-sensor_range
                Innov[1,0] = landmark_bearing-sensor_bearing
              
                # Normalize angle [-pi,pi]
                Innov[1,0] = Innov[1,0] % (2 * math.pi)
                if Innov[1,0] > math.pi:
                    Innov[1,0] = Innov[1,0] - 2 * math.pi

                # Compute self.x and covariance matrix (self.P)
                self.x = self.x + numpy.dot(R, Innov)
                self.P = self.P - numpy.dot(numpy.dot(R, H),self.P)

#------------------------------------------------------------------------------------------------------------------------

    def compute_state_transition_matrix(self,theta_k, vel_trans, t):
        F = numpy.eye(3)
        F[0, 2] = -t * vel_trans * math.sin(theta_k)
        F[1, 2] = t * vel_trans * math.cos(theta_k)
        return F

#------------------------------------------------------------------------------------------------------------------------

    def compute_measurement_matrix_range_bearing(self, landmark_x, landmark_y, x_pred, y_pred, theta_pred):
        
        # Predicted coordinates 
        x_r = x_pred
        y_r = y_pred
        theta_r = theta_pred

        # Landmark coordinates
        x_l = landmark_x
        y_l = landmark_y
        
        # Calculations
        delta_x = x_l - x_r
        delta_y = y_l - y_r
        q = (-delta_x)**2 + (-delta_y)**2
        sqrt_q = numpy.sqrt(q)
        sensor_range = sqrt_q
        sensor_bearing = math.atan2(delta_y, delta_x) - theta_r

        # Normalize sensor_bearing [-pi,pi]
        sensor_bearing = sensor_bearing % (2 * math.pi)
        if sensor_bearing > math.pi:
            sensor_bearing = sensor_bearing - 2 * math.pi

        H = numpy.zeros((2, 3))
        H[0, 0] = -delta_x / sqrt_q
        H[0, 1] = -delta_y / sqrt_q
        H[0, 2] = 0
        H[1, 0] = delta_y / (delta_x**2 + delta_y**2)
        H[1, 1] = -delta_x / (delta_x**2 + delta_y**2)
        H[1, 2] = -1

        return H, sensor_range, sensor_bearing

#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = float(self.x[0])
        est_msg.pose.y = float(self.x[1])
        est_msg.pose.theta = float(self.x[2])
        self.pub_est.publish(est_msg)

def main(args=None):
    rclpy.init(args=args)   
    est = Estimator()
    rclpy.spin(est)
                
if __name__ == '__main__':
   main()

 
