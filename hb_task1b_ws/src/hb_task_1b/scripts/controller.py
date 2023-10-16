import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

   def __init__(self):
        super().__init__('hb_task1b_controller')
        
        # Initialze Publisher and Subscriber
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Declare a Twist message
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0
        # Initialise the required variables to 0

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        self.x_current=0
        self.y_current=0
        self.theta_current=0

        self.Kp_linear = 0.5 
        self.Kp_angular = 0.5

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        # and also Kp values for the P Controller


        # client for the "next_goal" service
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
    
    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()
   
    # Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
                ####################################################

                if abs(ebot_controller.x_goal - ebot_controller.x_current) < 0.1 and \
                   abs(ebot_controller.y_goal - ebot_controller.y_current) < 0.1 and \
                   abs(ebot_controller.theta_goal - ebot_controller.theta_current) < 0.1:
                    ebot_controller.index += 1
                    if ebot_controller.flag == 1:
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)

                # Find error (in x, y and theta) in global frame
                # the /odom topic is giving pose of the robot in global frame
                # the desired pose is declared above and defined by you in global frame
                # therefore calculate error in global frame

                # (Calculate error in body frame)
                # But for Controller outputs robot velocity in robot_body frame, 
                # i.e. velocity are define is in x, y of the robot frame, 
                # Notice: the direction of z axis says the same in global and body frame
                # therefore the errors will have have to be calculated in body frame.
                # 
                # This is probably the crux of Task 1, figure this out and rest should be fine.

                # Finally implement a P controller 
                # to react to the error with velocities in x, y and theta.

                # Safety Check
                # make sure the velocities are within a range.
                # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
                # we may get away with skipping this step. But it will be very necessary in the long run.


                #If Condition is up to you
                
                ############     DO NOT MODIFY THIS       #########
                ebot_controller.index += 1
                if ebot_controller.flag == 1 :
                    ebot_controller.index = 0
                ebot_controller.send_request(ebot_controller.index)
                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(ebot_controller)
    
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
        main()
