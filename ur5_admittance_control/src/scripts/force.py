#!/usr/bin/env python3

import rospy
import sys
import copy
import moveit_msgs.msg
from geometry_msgs.msg import WrenchStamped
from math import sin
import moveit_commander



def apply_force_and_move():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('end_effector_force_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

    group_variable_values = group.get_current_joint_values()

    group.set_joint_value_target(group_variable_values)

    # Set the force to be applied in x, y, and z directions
    force_x = 40.0  # Adjust the force value as needed
    force_y = 0.0
    force_z = 0.0

    # Publish the force to the end effector
    force_publisher = rospy.Publisher('/wrench', WrenchStamped, queue_size=1)
    force_msg = WrenchStamped()
    force_msg.header.frame_id = "wrist_1_link"
    force_msg.wrench.force.x = force_x
    force_msg.wrench.force.y = force_y
    force_msg.wrench.force.z = force_z

    force_msg.wrench.force.x.group_variable_values[5]

    # Wait for the force publisher to connect to the topic
    while not force_publisher.get_num_connections() > 0:
        rospy.sleep(0.1)

    # Publish the force for 2 seconds
    duration = 20.0  # Adjust the duration as needed
    rate = rospy.Rate(10)  # 10 Hz (adjust as needed)
    end_time = rospy.get_time() + duration

    while rospy.get_time() < end_time:
        force_publisher.publish(force_msg)
        rate.sleep()

    # Stop applying the force
    force_msg.wrench.force.x = 0.0
    force_msg.wrench.force.y = 0.0
    force_msg.wrench.force.z = 0.0
    force_publisher.publish(force_msg)

    # Move the robot to the "home" position after applying the force
    group.set_named_target("home")
    plan = group.plan()
    group.execute(plan)
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        apply_force_and_move()
    except rospy.ROSInterruptException:
        pass

