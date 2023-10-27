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
        # We'll leave this for you to figure out the syntax for
        # initialising publisher and subscriber of cmd_vel and odom respectively
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Declare a Twist message
        self.vel = Twist()

        # Initialise the required variables to 0
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        # Initialise variables that may be needed for the control loop
        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        self.error_x=0.0
        self.error_y=0.0
        self.error_theta=0.0

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        # and also Kp values for the P Controller
        self.Kp_linear = 1.0
        self.Kp_angular = 1.0


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0

   def send_request(self, index):
        self.req.request_goal = self.index                 
        self.future = self.cli.call_async(self.req)

   def odom_callback(self,msg):
        x_current = msg.pose.pose.position.x
        y_current = msg.pose.pose.position.y

        # Calculate error in x and y
        self.error_x = self.x_goal - x_current
        self.error_y = self.y_goal - y_current

        # Calculate desired linear velocity
        self.vel.linear.x = self.Kp_linear * self.error_x
        self.vel.linear.y = self.Kp_linear * self.error_y

        # Extract current orientation and calculate error in theta
        orientation = msg.pose.pose.orientation
        _, _, theta_current = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.error_theta = self.theta_goal - theta_current

        # Calculate desired angular velocity
        self.vel.angular.z = self.Kp_angular * self.error_theta

   def euclidean_distance(self):
       return math.sqrt((self.error_x**2) + (self.error_y**2))

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
                ebot_controller.x_goal      = response.x_goal
                ebot_controller.y_goal      = response.y_goal
                ebot_controller.theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
                ####################################################    

                rclpy.spin_once(ebot_controller)
                print(ebot_controller.x_goal)
                print(ebot_controller.y_goal)
                print(ebot_controller.index)
                print(ebot_controller.flag)
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
                if ebot_controller.euclidean_distance()>=0.5:
                    ebot_controller.pub_cmd_vel.publish(ebot_controller.vel)
                    continue
                    
                else :
                    ebot_controller.vel.linear.x=0.0
                    ebot_controller.vel.linear.y=0.0
                    ebot_controller.vel.angular.z=0.0
                    ebot_controller.pub_cmd_vel.publish(ebot_controller.vel)
                    time.sleep(1)

                

                ############     DO NOT MODIFY THIS       #########
                ebot_controller.index += 1
                if ebot_controller.flag == 1 :
                    ebot_controller.index = 0
                ebot_controller.send_request(ebot_controller.index)
                ####################################################    
        # Spin once to process callbacks
        #time.sleep(1)
        rclpy.spin_once(ebot_controller)

    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
