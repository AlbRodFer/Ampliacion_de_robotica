import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'desired_joint_accelerations', 10)
        self.timer = self.create_timer(0.001, self.publish_acceleration)  # 1kHz frequency
        self.t = 0
        self.t_final = 4 
        self.theta0 = np.array([0, 0])  # Initial angles for joint 1 and joint 2
        self.thetaf = np.array([10, 5])  # Final angles for joint 1 and joint 2
        self.thetaff = self.thetaf * (2 * np.pi) / 360
        self.a0 = self.theta0
        self.a1 = np.zeros(2)
        self.a2 = (3 / self.t_final**2) * (self.thetaff - self.theta0)
        self.a3 = -(2 / self.t_final**3) * (self.thetaff - self.theta0)

    def publish_acceleration(self):
        t = self.t
        if t > self.t_final:
            msg = Float64MultiArray()
            msg.data = [0.0, 0.0]
            self.publisher_.publish(msg)
            self.get_logger().info('Trajectory completed. Shutting down node.')
            rclpy.shutdown()
        else:
            a = 2 * self.a2 + 6 * self.a3 * t
            msg = Float64MultiArray()
            msg.data = a.tolist()
            self.publisher_.publish(msg)
            self.t += 0.001

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()

if __name__ == '__main__':
    main()