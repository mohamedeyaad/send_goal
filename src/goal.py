#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Initialize global variables
current_pose = Pose()

def pose_callback(data):
    global current_pose
    current_pose = data

def move_to_goal(x_goal, y_goal):
    global current_pose

    rospy.init_node('turtle_move_to_goal', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)

    goal_pose = Pose()
    goal_pose.x = x_goal
    goal_pose.y = y_goal

    vel_msg = Twist()

    while not rospy.is_shutdown():
        # Calculate the distance to the goal
        distance = math.sqrt((goal_pose.x - current_pose.x)**2 + (goal_pose.y - current_pose.y)**2)

        # Calculate the linear velocity
        vel_msg.linear.x = 1.5 * distance

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)
        angle_diff = angle_to_goal - current_pose.theta

        # Calculate the angular velocity
        vel_msg.angular.z = 4.0 * angle_diff

        # Stop the turtle when it reaches the goal
        if distance < 0.01:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            break

        # Publish the velocity message
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        x_goal = 5.0  # Set your x goal here
        y_goal = 5.0  # Set your y goal here
        move_to_goal(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass
