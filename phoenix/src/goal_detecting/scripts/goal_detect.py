#!/usr/bin/env python

"""
Module Name: Goal Detection

This module is for detecting goals in a ROS (Robot Operating System) environment.
It subscribes to the topic 'objectsStamped' where it receives objects data, checks if a goal is found, 
and then controls the movement of a turtle robot accordingly by publishing to '/turtle1/cmd_vel'. 
If a goal is found, it sets a linear and angular velocity for the robot to move towards the goal, otherwise, 
the robot stays still.
"""

import rospy
from geometry_msgs.msg import Twist
from find_object_2d.msg import ObjectsStamped

class DetectGoal:
  def __init__(self):
    # Publish to turtle1's cmd_vel topic
    self.cmd_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 1)
    
    # Subscribe to objectsStamped topic
    self.find_sub = rospy.Subscriber("objectsStamped", ObjectsStamped, self.callback, queue_size = 1)
    
    self.found = False  # Variable to hold the state of the goal (found or not)
    self.rate = rospy.Rate(5)  # Set rate of 5 Hz

  def callback(self, msg):
    # Check if any object is detected and set the 'found' variable accordingly
    if len(msg.objects.data) > 0:
      self.found = True
    else:
      self.found = False
    print(msg, self.found)  # Print the message and the state of the 'found' variable

  def spin(self):
    vel_msg = Twist()
    while not rospy.is_shutdown():  # Loop until ROS is shutdown
      if self.found:  # If goal is found, set linear and angular velocity
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = 0.3
      else:  # Else, stop the robot
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

      # Publish the velocity message
      self.cmd_pub.publish(vel_msg)
      
      # Sleep to maintain the rate of 5Hz
      self.rate.sleep()

if __name__ == "__main__":
  rospy.init_node("goaldetect_node")  # Initialize the ROS node
  r = DetectGoal()  # Create an object of the DetectGoal class
  r.spin()  # Call the spin method

