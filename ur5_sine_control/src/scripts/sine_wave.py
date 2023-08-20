#! /usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
from math import sin

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ur5_sine_control_node', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)
rate = rospy.Rate(10)

group_variable_values = group.get_current_joint_values()

while not rospy.is_shutdown():
        # Time parameter for the sine wave (adjust as needed)
    t = rospy.Time.now().to_sec()
    group_variable_values[0] = sin(t)
    group_variable_values[1] = sin(2*t)
    group_variable_values[3] = sin(3*t)
    group_variable_values[5] = sin(4*t)

    group.set_joint_value_target(group_variable_values)
    plan2 = group.plan()

    group.go(wait=True)
    


    rate.sleep()

moveit_commander.roscpp_shutdown()


