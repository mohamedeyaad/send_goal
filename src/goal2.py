#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Initialize global variables for both turtles
current_pose_turtle1 = Pose()
current_pose_turtle2 = Pose()

def pose_callback_turtle1(data):
    global current_pose_turtle1
    current_pose_turtle1 = data

def pose_callback_turtle2(data):
    global current_pose_turtle2
    current_pose_turtle2 = data

def move_to_goal(turtle, x_goal, y_goal):
    global current_pose_turtle1, current_pose_turtle2

    pub = rospy.Publisher(f'/{turtle}/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    goal_pose = Pose()
    goal_pose.x = x_goal
    goal_pose.y = y_goal

    vel_msg = Twist()

    while not rospy.is_shutdown():
        if turtle == "turtle1":
            current_pose = current_pose_turtle1
        else:
            current_pose = current_pose_turtle2

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
        rospy.init_node('turtle_move_to_goal', anonymous=True)

        # Subscribers for both turtles
        sub_turtle1 = rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
        sub_turtle2 = rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)

        # Move turtle1 to its goal
        x_goal_turtle1 = 5.0  # Set your x goal here
        y_goal_turtle1 = 5.0  # Set your y goal here
        move_to_goal("turtle1", x_goal_turtle1, y_goal_turtle1)

        # Move turtle2 to its goal
        x_goal_turtle2 = 8.0  # Set your x goal here
        y_goal_turtle2 = 8.0  # Set your y goal here
        move_to_goal("turtle2", x_goal_turtle2, y_goal_turtle2)

    except rospy.ROSInterruptException:
        pass
