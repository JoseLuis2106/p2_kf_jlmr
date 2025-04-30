import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion
from irobot_create_msgs.msg import WheelVels
from tf_transformations import quaternion_from_euler

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator, generate_noisy_measurement
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1

        self.mu = np.zeros(3)

        obs_noise_std=[0.02, 0.02, 0.02]

        self.kf = KalmanFilter(initial_state, initial_covariance,obs_noise_std)
        
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

        self.seq=0
        self.prev_time = None

    def odom_callback(self, msg):
        # TODO: Extract velocities and timestep
        (x, y, yaw) = odom_to_pose2D(msg)

        curr_time=self.get_clock().now().nanoseconds
        if self.prev_time:
            dt=(curr_time - self.prev_time) / 1e9
        else:
            dt=0
        self.prev_time = curr_time
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        [x, y, yaw, v, omega] = generate_noisy_measurement((x, y, yaw), v, omega)

        u=np.array([v, omega])

        # TODO: Run predict() and update() of KalmanFilter
        _, self.kf.B = velocity_motion_model(self.mu, dt)
        self.kf.predict(u, dt)
        self.mu, self.sigma = self.kf.update(np.array([x, y, yaw]))

        # TODO: Publish estimated state
        msg_out = PoseWithCovarianceStamped()
        msg_out.header.seq = self.seq
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = "odom"
        msg_out.pose.pose.position.x = self.mu[0]
        msg_out.pose.pose.position.y = self.mu[1]
        msg_out.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, self.mu[2])
        msg_out.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        cov = np.zeros((6, 6))
        cov[0:3, 0:3] = self.sigma
        msg_out.pose.covariance = cov.flatten().tolist()

        self.publisher.publish(msg_out)
        
        self.seq += 1
        
        self.visualizer.update(self.normalized_pose, self.mu, self.sigma, step="update")


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
