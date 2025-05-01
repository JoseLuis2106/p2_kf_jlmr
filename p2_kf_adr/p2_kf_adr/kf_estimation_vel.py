import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer
from .motion_models import velocity_motion_model_2


class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.mu = np.zeros(6)

        proc_noise_std=[0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
        # proc_noise_std=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

        obs_noise_std=[0.02, 0.02, 0.02, 0.02, 0.02, 0.02]
        # obs_noise_std=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

        self.kf = KalmanFilter_2(initial_state, initial_covariance, proc_noise_std, obs_noise_std)
        
        self.visualizer = Visualizer()

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf2_estimate',
            10
        )

        self.prev_time = None

    def odom_callback(self, msg):
        # TODO: Extract position, orientation, velocities from msg
        (x, y, yaw) = odom_to_pose2D(msg)
        self.normalized_pose = get_normalized_pose2D(self.initial_state,(x,y,yaw))
        (x, y, yaw) = self.normalized_pose

        curr_time=self.get_clock().now().nanoseconds
        if self.prev_time:
            dt=(curr_time - self.prev_time) / 1e9
        else:
            dt=0
        self.prev_time = curr_time
        
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        vx = np.cos(yaw)*v 
        vy = np.sin(yaw)*v

        [x, y, yaw, vx, vy, omega] = generate_noisy_measurement_2((x, y, yaw), vx, vy, omega)
        

        # u=np.array([v, omega])

        # TODO: Run predict() and update() of KalmanFilter_2
        self.kf.predict(None, dt)
        self.mu, self.sigma = self.kf.update(np.array([x, y, yaw, vx, vy, omega]))

        # TODO: Publish estimated full state
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
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

