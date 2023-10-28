import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
import sys

class HBTask1BController(Node):
    def __init__(self):
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

        self.stage=1  
        self.goalReached=True

    def odom_callback(self,msg):
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.theta_current = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])


    def go_to_goal(self):
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
        kp = 10
        kp_linear=7

        error_theta=self.theta_goal-self.theta_current

        if self.stage==1:
            if (distance_to_goal)<distance_tolerance:
                self.stage+=1
            elif abs(angle_error) > angle_tolerance:
                new_vel.angular.z = kp * angle_error
            else :
                # print("Hey")
                if( distance_to_goal ) >= distance_tolerance:
                    new_vel.linear.x = kp_linear * distance_to_goal
                else :
                    new_vel.linear.x= 0.0
                    new_vel.angular.z=0.0
                    new_vel.linear.y=0.0
                    self.stage+=1

        if self.stage==2:
            if abs(error_theta)>angle_tolerance:
                new_vel.angular.z = kp * error_theta
            else:
                new_vel.angular.z = 0.0
                self.stage+=1

        if self.stage==3:
            new_vel.linear.x= 0.0
            new_vel.angular.z=0.0
            new_vel.linear.y=0.0
            self.pub_cmd_vel.publish(new_vel)
            self.get_logger().info("Goal Reached ")
            self.goalReached=True

        self.pub_cmd_vel.publish(new_vel)
                

def main(args=None):
    rclpy.init(args=args)
    ebot = HBTask1BController()
    ebot.req.request_goal = ebot.index                 
    ebot.future = ebot.cli.call_async(ebot.req)
    while rclpy.ok():
        rclpy.spin_until_future_complete(ebot, ebot.future)
        if ebot.future.result() is not None:
            ebot.goalReached=False
            ebot.x_goal=ebot.future.result().x_goal
            ebot.y_goal=ebot.future.result().y_goal
            ebot.theta_goal=ebot.future.result().theta_goal
            # print(ebot.x_goal)
            # print(ebot.y_goal)
            # print(ebot.theta_goal)
            # print(ebot.future.result().end_of_list)
            while not ebot.goalReached:
                rclpy.spin_once(ebot)
            time.sleep(2)
            ebot.index+=1
            ebot.stage=1
            if ebot.future.result().end_of_list:
                ebot.index=0
            ebot.req.request_goal = ebot.index                 
            ebot.future = ebot.cli.call_async(ebot.req)
        
    ebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()