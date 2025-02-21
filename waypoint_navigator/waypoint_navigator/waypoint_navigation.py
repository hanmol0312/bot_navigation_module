import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        self.declare_parameter('waypoint_1_x', 1.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 2.0)
        self.declare_parameter('waypoint_2_y', 2.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.current_waypoint_idx = 0
        self.reached_goal = False
        
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid_linear = PIDController(kp, ki, kd)
        self.pid_angular = PIDController(kp, ki, kd)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
    
    def euler_from_quaternion(self, q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)
        return (0.0, 0.0, yaw)
    
    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return
        
        wp_x, wp_y = self.waypoints[self.current_waypoint_idx]
        distance = math.sqrt((wp_x - self.x)**2 + (wp_y - self.y)**2)
        angle_to_target = math.atan2(wp_y - self.y, wp_x - self.x)
        angle_error = angle_to_target - self.yaw
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if distance < 0.1:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.reached_goal = True
            return
        
        linear_velocity = self.pid_linear.compute(distance, dt)
        angular_velocity = self.pid_angular.compute(angle_error, dt)
        
        twist = Twist()
        twist.linear.x = min(0.5, max(-0.5, linear_velocity))
        twist.angular.z = min(1.0, max(-1.0, angular_velocity))
        self.cmd_vel_pub.publish(twist)
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Robot has reached both waypoints.")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
