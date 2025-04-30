import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, Quaternion

import numpy as np

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, generate_noisy_measurement_2
from .filters.kalman_filter import KalmanFilter_2
from .visualization import Visualizer


class KalmanFilterPureNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_pure_node')

        # TODO: Initialize 6D state and covariance
        initial_state = np.zeros(6)
        initial_covariance = np.eye(6) * 0.1

        self.kf = KalmanFilter_2(initial_state, initial_covariance)

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

        self.seq=0
        self.prev_time = None

    def odom_callback(self, msg):
        # TODO: Extract position, orientation, velocities from msg
        (x, y, yaw) = odom_to_pose2D(msg)

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
        self.kf.A, _ = velocity_motion_model(dt)
        self.kf.predict(None, dt)
        self.mu, self.sigma = self.kf.update(np.array([x, y, yaw, vx, vy, omega]))

        # TODO: Publish estimated full state
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
        cov[0:6, 0:6] = self.sigma
        msg_out.pose.covariance = cov.flatten().tolist()

        self.publisher.publish(msg_out)
        
        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterPureNode()
    rclpy.spin(node)
    rclpy.shutdown()

