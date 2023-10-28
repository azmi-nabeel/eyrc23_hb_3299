'''
# Team ID:          eYRC#HB#3299
# Theme:            Hologlyph Bots
# Author List:      Azmi Nabeel
# Filename:         hb_task_1b_3299.py
# Functions:        __init__(constructor), odom_callback, go_to_goal, main
# Global variables: None
'''

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
        '''
Purpose:
---
constructor for class HBTask1BController

Example call:
---
Called automatically when an instance of class HBTask1BController is created

'''
        super().__init__('hb_task1b_controller')

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            print("'next_goal' service is not available, waiting...")

        self.timer=self.create_timer(0.1,self.go_to_goal)

        self.x_current=0.0
        self.y_current=0.0
        self.theta_current=0.0

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        self.stage=1  # counter for the stages of go_to_goal
        #go_to_goal happens in 3 stages 1)going to goal location 2)getting the goal orientation 3)stabilising and pausing


        self.goalReached=True

    def odom_callback(self,msg):
        '''
Purpose:
---
Callback function for Odometry subscriber.

Input Arguments:
---
`msg` :  Odometry class
    The message obtained in the odom topic by Odometry subscriber

Returns:
---
None

Example call:
---
Called as a callback when a msg is recieved in the odom topic by theodometry subscriber
'''

        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.theta_current = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


    def go_to_goal(self):
        '''
Purpose:
---
Function to lead the bot to the goal provided by publishing appropriate velocity to cmd_vel topic

Input Arguments:
---


Returns:
---
None

Example call:
---
Called in the timer callback
'''
        #if goal is reached do nothing and return
        if self.goalReached:
            return
        
        new_vel=Twist()

        # Ecludian Distance
        distance_to_goal = math.sqrt( (self.x_goal - self.x_current)**2  + (self.y_goal - self.y_current)**2 )
        # Angle to Goal
        angle_to_goal =math.atan2(self.y_goal - self.y_current , self.x_goal - self.x_current)

        distance_tolerance = 0.1
        angle_tolerance = 0.1

        angle_error = angle_to_goal - self.theta_current

        #constants to regulate the velocity 
        kp = 10
        kp_linear=7

        error_theta=self.theta_goal-self.theta_current

        #stage1 - going to the goal position x and y coordinates
        if self.stage==1:
            if (distance_to_goal)<distance_tolerance:
                self.stage+=1
            elif abs(angle_error) > angle_tolerance:
                new_vel.angular.z = kp * angle_error
            else :
                if( distance_to_goal ) >= distance_tolerance:
                    new_vel.linear.x = kp_linear * distance_to_goal
                else :
                    new_vel.linear.x= 0.0
                    new_vel.angular.z=0.0
                    new_vel.linear.y=0.0
                    self.stage+=1

        #stage2 - getting the required goal orientation
        if self.stage==2:
            if abs(error_theta)>angle_tolerance:
                new_vel.angular.z = kp * error_theta
            else:
                new_vel.angular.z = 0.0
                self.stage+=1

        #stage3 - stopping and stabilising once the goal is reached and set the goalReached flag to true
        if self.stage==3:
            new_vel.linear.x= 0.0
            new_vel.angular.z=0.0
            new_vel.linear.y=0.0
            self.pub_cmd_vel.publish(new_vel)
            self.get_logger().info("Goal Reached ")
            self.goalReached=True

        self.pub_cmd_vel.publish(new_vel)
                

def main(args=None):
    '''
Purpose:
---
Main function to create instance of HBTask1BController and make requests for goals from the service node

Input Arguments:
---
None

Returns:
---
None

Example call:
---
Called by the system (main function)
'''

    rclpy.init(args=args)
    ebot = HBTask1BController()

    #making an initial request for goal
    ebot.req.request_goal = ebot.index                 
    ebot.future = ebot.cli.call_async(ebot.req)

    #Loop to run continously till interrupted
    while rclpy.ok():
        rclpy.spin_until_future_complete(ebot, ebot.future)

        #if the request is fulfilled
        if ebot.future.result() is not None:
            ebot.goalReached=False                             #sets the goalReached flag to false as new goal is obtained
            ebot.x_goal=ebot.future.result().x_goal
            ebot.y_goal=ebot.future.result().y_goal
            ebot.theta_goal=ebot.future.result().theta_goal    #getting the goal parameters

            #till the goal is reached keep spiining once to process callbacks and run go_to_goal
            while not ebot.goalReached:
                rclpy.spin_once(ebot)

            #once the goal is reached and stabilised pausing for 2 secs
            time.sleep(2)

            #incrementing the goal index and reinitialising stage to 1 for the new goal
            ebot.index+=1
            ebot.stage=1
            if ebot.future.result().end_of_list:
                ebot.index=0

            #request for new goal
            ebot.req.request_goal = ebot.index                 
            ebot.future = ebot.cli.call_async(ebot.req)
        
    ebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()