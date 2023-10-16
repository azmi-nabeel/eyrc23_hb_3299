import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')

        # Initialize Publisher and Subscriber
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Declare a Twist message
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0

        # Initialize variables for desired goal-pose and P controller gains
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        self.Kp_linear = 0.5  # P controller gain for linear velocity
        self.Kp_angular = 0.5  # P controller gain for angular velocity

        # Client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')
        self.req = NextGoal.Request()
        self.index = 0

    def send_request(self, index):
        self.req.index = index
        self.future = self.cli.call_async(self.req)

    def odom_callback(self, msg):
        # Extract current pose from odometry message
        x_current = msg.pose.pose.position.x
        y_current = msg.pose.pose.position.y

        # Calculate error in x and y
        error_x = self.x_goal - x_current
        error_y = self.y_goal - y_current

        # Calculate desired linear velocity
        self.vel.linear.x = self.Kp_linear * error_x
        self.vel.linear.y = self.Kp_linear * error_y

        # Extract current orientation and calculate error in theta
        orientation = msg.pose.pose.orientation
        _, _, theta_current = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        error_theta = self.theta_goal - theta_current

        # Calculate desired angular velocity
        self.vel.angular.z = self.Kp_angular * error_theta

        # Publish the control command
        self.pub_cmd_vel.publish(self.vel)

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the HBTask1BController class
    hb_controller = HBTask1BController()

    # Send an initial request with the index from hb_controller.index
    hb_controller.send_request(hb_controller.index)

    # Main loop
    while rclpy.ok():
        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # Response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().info('Service call failed %r' % (e,))
            else:
                # Update goal pose
                hb_controller.x_goal = response.x_goal
                hb_controller.y_goal = response.y_goal
                hb_controller.theta_goal = response.theta_goal
                hb_controller.flag = response.end_of_list

                # If condition to check if goal is reached (e.g., using a threshold)
                # If goal is reached, send a new request for the next goal
                if abs(hb_controller.x_goal - x_current) < threshold and \
                   abs(hb_controller.y_goal - y_current) < threshold and \
                   abs(hb_controller.theta_goal - theta_current) < threshold:
                    hb_controller.index += 1
                    if hb_controller.flag == 1:
                        hb_controller.index = 0
                    hb_controller.send_request(hb_controller.index)

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)

    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
