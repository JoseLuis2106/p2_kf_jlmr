import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator, generate_noisy_measurement
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 
from .motion_models import velocity_motion_model

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        self.initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1

        self.mu = np.zeros(3)

        proc_noise_std=[0.02, 0.02, 0.02]
        # proc_noise_std=[0.02, 0.02, 0.02]

        obs_noise_std=[0.02, 0.02, 0.02]
        # obs_noise_std=[0.02, 0.02, 0.02]

        self.kf = KalmanFilter(self.initial_state, initial_covariance, proc_noise_std, obs_noise_std)
        
        self.visualizer = Visualizer()

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

        self.prev_time = None

    def odom_callback(self, msg):
        # TODO: Extract velocities and timestep
        (x, y, yaw) = odom_to_pose2D(msg)
        self.normalized_pose = get_normalized_pose2D(self.initial_state,(x,y,yaw))
        (x, y, yaw) = self.normalized_pose

        curr_time=self.get_clock().now().nanoseconds
        if self.prev_time:
            dt = (curr_time - self.prev_time) / 1e9
        else:
            dt = 0
        self.prev_time = curr_time
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        [x, y, yaw, v, omega] = generate_noisy_measurement((x, y, yaw), v, omega)

        u=np.array([v, omega])

        # TODO: Run predict() and update() of KalmanFilter
        self.kf.predict(u, dt)
        self.mu, self.sigma = self.kf.update(np.array([x, y, yaw]))

        # TODO: Publish estimated state
        msg_out = PoseWithCovarianceStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = "map"
        msg_out.pose.pose.position.x = self.mu[0]
        msg_out.pose.pose.position.y = self.mu[1]
        msg_out.pose.pose.position.z = 0.0
        msg_out.pose.pose.orientation.z = math.sin(self.mu[2]/2)
        msg_out.pose.pose.orientation.w = math.cos(self.mu[2]/2)
        msg_out.pose.covariance = [0.0] * 36
        msg_out.pose.covariance[0] = self.sigma[0, 0]
        msg_out.pose.covariance[1] = self.sigma[0, 1]
        msg_out.pose.covariance[5] = self.sigma[0, 2]
        msg_out.pose.covariance[6] = self.sigma[1, 0]
        msg_out.pose.covariance[7] = self.sigma[1, 1]
        msg_out.pose.covariance[11] = self.sigma[1, 2]
        msg_out.pose.covariance[30] = self.sigma[2, 0]
        msg_out.pose.covariance[31] = self.sigma[2, 1]
        msg_out.pose.covariance[35] = self.sigma[2, 2]

        self.publisher.publish(msg_out)
        
        self.visualizer.update(self.normalized_pose, self.mu, self.sigma, step="update")
        
        return self.mu, self.sigma


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
