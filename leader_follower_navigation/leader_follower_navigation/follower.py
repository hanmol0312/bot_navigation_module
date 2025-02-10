import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class FollowerBot(Node):
    def __init__(self):
        super().__init__('follower_bot_node')

        self.declare_parameter('safe_distance', 1.0)
        self.safe_distance = self.get_parameter('safe_distance').value
        self.orientation_tolerance = 0.4  

        self.leader_odom_sub = self.create_subscription(
            Odometry,
            '/leader_bot/odom',
            self.leader_odom_callback,
            10
        )

        self.follower_odom_sub = self.create_subscription(
            Odometry,
            '/follower_bot/odom',
            self.follower_odom_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/follower_bot/cmd_vel', 10)

        self.leader_pose = None
        self.follower_pose = None

    def leader_odom_callback(self, msg):
        self.leader_pose = msg.pose.pose
        self.follow_leader()

    def follower_odom_callback(self, msg):
        self.follower_pose = msg.pose.pose

    def follow_leader(self):
        if self.leader_pose is None or self.follower_pose is None:
            return

        dx = self.leader_pose.position.x - self.follower_pose.position.x
        dy = self.leader_pose.position.y - self.follower_pose.position.y
        distance = math.hypot(dx, dy)

        leader_orientation = euler_from_quaternion([
            self.leader_pose.orientation.x,
            self.leader_pose.orientation.y,
            self.leader_pose.orientation.z,
            self.leader_pose.orientation.w
        ])
        follower_orientation = euler_from_quaternion([
            self.follower_pose.orientation.x,
            self.follower_pose.orientation.y,
            self.follower_pose.orientation.z,
            self.follower_pose.orientation.w
        ])
 
        angle_to_leader = math.atan2(dy, dx)
        angle_diff = angle_to_leader - follower_orientation[2] + math.pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        cmd = Twist()
        print(angle_diff)
        if abs(angle_diff) > self.orientation_tolerance:
            cmd.angular.z = 0.5 * angle_diff
        elif distance > self.safe_distance:
            cmd.linear.x = 0.5 * (distance - self.safe_distance)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
