import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
import math
import numpy as np


class LocalizationNode(Node):
    """
    EKF localization node for TurtleBot3 Burger.

    State vector: [x, y, theta, v, omega]
    Fuses wheel encoder odometry (/joint_states) and IMU (/imu).
    Publishes pose estimate directly to /localization/pose (PoseStamped).
    """

    def __init__(self):
        super().__init__('localization_node')

        # TurtleBot3 Burger constants
        self.R = 0.033        # wheel radius (m)
        self.L = 0.165        # wheelbase (m) — slightly wider than spec for best results
        self.tau = 0.8        # velocity lag time constant (s)

        # EKF state: [x, y, theta, v, omega]
        self.n = 5
        self.x = np.zeros(self.n)
        self.P = np.eye(self.n) * 0.1

        # Noise matrices
        self.Q = np.diag([1e-10, 1e-10, 1e-9, 0.005, 0.005])
        self.R_imu = np.diag([0.001, 1.0])
        self.R_wheels = np.diag([0.05, 0.05])

        self.last_phi_l, self.last_phi_r = None, None
        self.v_cmd, self.omega_cmd = 0.0, 0.0
        self.last_time = None
        self.last_wheels_time = None
        self.last_imu_time = None

        path_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/localization/pose', 10)
        self.path_pub = self.create_publisher(Path, '/localization/path', path_qos)
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('EKF Localization Node started. Publishing to /localization/pose')

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def make_pd(self, P):
        P = (P + P.T) / 2.0
        ee, ev = np.linalg.eigh(P)
        ee = np.maximum(ee, 1e-9)
        return ev @ np.diag(ee) @ ev.T

    def f(self, x, dt):
        """Motion model: predict next state given current state and commands."""
        x_new = np.copy(x)
        x_new[0] += x[3] * math.cos(x[2]) * dt
        x_new[1] += x[3] * math.sin(x[2]) * dt
        x_new[2] = self.normalize_angle(x[2] + x[4] * dt)
        x_new[3] += (self.v_cmd - x[3]) * (dt / self.tau)
        x_new[4] += (self.omega_cmd - x[4]) * (dt / self.tau)
        return x_new

    def get_jacobian_f(self, x, dt):
        """Jacobian of motion model with respect to state."""
        F = np.eye(self.n)
        F[0, 2] = -x[3] * math.sin(x[2]) * dt
        F[0, 3] = math.cos(x[2]) * dt
        F[1, 2] = x[3] * math.cos(x[2]) * dt
        F[1, 3] = math.sin(x[2]) * dt
        F[2, 4] = dt
        F[3, 3] = 1.0 - (dt / self.tau)
        F[4, 4] = 1.0 - (dt / self.tau)
        return F

    def predict(self, dt):
        self.x = self.f(self.x, dt)
        F = self.get_jacobian_f(self.x, dt)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, H, h_x, R):
        """Standard EKF update step."""
        y = z - h_x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.x[2] = self.normalize_angle(self.x[2])
        self.P = self.make_pd((np.eye(self.n) - K @ H) @ self.P)

    def joint_state_callback(self, msg):
        try:
            l_idx = msg.name.index('wheel_left_joint')
            r_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            return

        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = curr_time
        if self.last_wheels_time is None:
            self.last_wheels_time = curr_time
            self.last_phi_l = msg.position[l_idx]
            self.last_phi_r = msg.position[r_idx]
            return

        dt = curr_time - self.last_time
        self.last_time = curr_time
        self.predict(dt)

        dt_w = curr_time - self.last_wheels_time
        self.last_wheels_time = curr_time
        if dt_w < 1e-6:
            return

        v_l = self.R * (msg.position[l_idx] - self.last_phi_l) / dt_w
        v_r = self.R * (msg.position[r_idx] - self.last_phi_r) / dt_w
        self.last_phi_l = msg.position[l_idx]
        self.last_phi_r = msg.position[r_idx]

        z = np.array([(v_l + v_r) / 2.0, (v_r - v_l) / self.L])
        H = np.zeros((2, self.n))
        H[0, 3] = 1.0
        H[1, 4] = 1.0
        h_x = np.array([self.x[3], self.x[4]])
        self.update(z, H, h_x, self.R_wheels)
        self.publish()

    def imu_callback(self, msg):
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = curr_time
        if self.last_imu_time is None:
            self.last_imu_time = curr_time
            return

        dt = curr_time - self.last_time
        self.last_time = curr_time
        self.predict(dt)
        self.last_imu_time = curr_time

        ax_expected = (self.v_cmd - self.x[3]) / self.tau
        z = np.array([msg.angular_velocity.z, msg.linear_acceleration.x])
        H = np.zeros((2, self.n))
        H[0, 4] = 1.0
        H[1, 3] = -1.0 / self.tau
        h_x = np.array([self.x[4], ax_expected])
        self.update(z, H, h_x, self.R_imu)
        self.publish()

    def cmd_vel_callback(self, msg):
        self.v_cmd = msg.twist.linear.x
        self.omega_cmd = msg.twist.angular.z

    def publish(self):
        now = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose.position.x = self.x[0]
        pose_msg.pose.position.y = self.x[1]
        pose_msg.pose.orientation.z = math.sin(self.x[2] / 2)
        pose_msg.pose.orientation.w = math.cos(self.x[2] / 2)
        self.pose_pub.publish(pose_msg)

        self.path.poses.append(pose_msg)
        self.path.header.stamp = now
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
