import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.01)
        
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.current_waypoint_idx = 0
        self.reached_goal = False
        self.odom_callback_group = MutuallyExclusiveCallbackGroup()
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.pid_linear = PIDController(kp, ki, kd)
        self.pid_angular = PIDController(kp, ki, kd)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10,callback_group=self.odom_callback_group)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    
    
    def control_loop(self):
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return
        
        wp_x, wp_y = self.waypoints[self.current_waypoint_idx]
        distance = math.sqrt((wp_x - self.x)**2 + (wp_y - self.y)**2)
        angle_to_target = math.atan2(wp_y - self.y, wp_x - self.x)
        angle_error = angle_to_target - self.yaw
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if distance < 0.1:
            self.current_waypoint_idx += 1
            print("Reached 1")
            
            if self.current_waypoint_idx >= len(self.waypoints):
                self.reached_goal = True
            return
        
        
        linear_velocity = self.pid_linear.compute(distance, dt)
        angular_velocity = self.pid_angular.compute(angle_error, dt)
        print(f"angle_error:{angle_error}, yaw:{self.yaw}, angle_target:{angle_to_target}")
        twist = Twist()
        twist.linear.x = min(0.3, max(-0.5, linear_velocity))
        twist.angular.z = min(0.8, max(-0.5, angular_velocity))
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
