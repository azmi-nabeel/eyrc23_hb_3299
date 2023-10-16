########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID:eYRC#HB#3299
# Team Leader Name:Azmi Nabeel
# Team Members Name:Abhijay, Pratush Rai, Hitesh Kumar
# College:Indian Institute of Information Technology, Dharwad.
########################################################################################################################

'''
# Team ID:          eYRC#HB#3299
# Theme:            Hologlyph Bots
# Author List:      Azmi Nabeel
# Filename:         task_1a_3299.py
# Functions:        main, publish_velocity, spawn_new_turtle
# Global variables: iteration_count, has_circle1_completed
'''



import rclpy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

iteration_count=0
has_circle1_completed=False

def main():
    '''
Purpose:
---
Main function to initialise the node, create publishers and call function to publish velocity to draw circles.

Input Arguments:
---
None

Returns:
---
None

Example call:
---
Called automatically when doing ros2 run...
'''

    rclpy.init()
    node = rclpy.create_node('turtlecircle')

    publisher1 = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    publisher2 = node.create_publisher(Twist, '/turtle2/cmd_vel', 10)
    timer_period = 0.1 
    timer = node.create_timer(timer_period, lambda: publish_velocity(publisher1,publisher2,node,3.0,2.5))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def publish_velocity(publisher1,publisher2,node,linear_v,angular_v):
    '''
Purpose:
---
This function publishes twist messages to control either the first or the second turtle depending on the flag variable has_circle1_completed.
If the flag is false, twist msgs are published to turtle1/cmd_vel to draw a circle. 
When the flag is true, the twist msgs are published to turtle2/cmd_vel to draw a bigger circle.
Circle is drawn by repeating a linear velocity and angular velocity 25 times.
Circle is made bigger by varying linear velocity.

Input Arguments:
---
`publisher1` :  []
    defines publisher for turtle1 cmd_vel topic

`publisher2` :  []
    defines publisher for turtle2 cmd_vel topic

`linear_v` :  [float]
    passes the linear velocity to be used to draw the circle

`angular_v` :  [float]
    passes the angular velocity to be used to draw the circle


Returns:
---
None

Example call:
---
publish_velocity(publisher1,publisher2,node,3.0,2.5)
'''

    vel = Twist()
    vel.linear.x = linear_v
    vel.linear.y = 0.0
    vel.linear.z = 0.0

    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = angular_v

    global iteration_count,has_circle1_completed

    iteration_count+=1
    if has_circle1_completed==False:
        publisher1.publish(vel)
        if iteration_count>25:
            has_circle1_completed=True
            iteration_count=0
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            publisher1.publish(vel)
            node.get_logger().info('Smaller circle completed. Stopping turtle1.')
            spawn_new_turtle(node)

    if has_circle1_completed:
        vel.linear.x = linear_v+2
        vel.angular.z = -angular_v
        publisher2.publish(vel)
        if iteration_count>25:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            publisher2.publish(vel)
            node.get_logger().info('Bigger circle completed. Stopping turtle2.')
            node.destroy_node()
            rclpy.shutdown()

def spawn_new_turtle(node):
    '''
Purpose:
---
Spawns a new turtle at the same starting position as turtle1

Input Arguments:
---
`node` :  []
    current node.

Returns:
---
None

Example call:
---
spawn_new_turtle(node)
'''

    client = node.create_client(Spawn, '/spawn')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    req = Spawn.Request()
    req.x = 5.197917
    req.y = 5.197917
    req.theta = 0.0
    req.name = 'turtle2' 

    future = client.call_async(req)

    node.get_logger().info('turtle2 spawned..')

if __name__ == '__main__':
    main()