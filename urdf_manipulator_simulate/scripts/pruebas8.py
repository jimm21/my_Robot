#! /usr/bin/env python
import sys
from math import pi
import rospy
import moveit_commander
import moveit_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('trajectory_control', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("robot_arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


home_pose = group.get_current_pose().pose
print("up", home_pose)

# We can get the joint values from the group and adjust some of the values:
joint_goal = group.get_current_joint_values()
joint_goal[0] = -pi/2
joint_goal[1] = 0.0
joint_goal[2] = pi/2

group.go(joint_goal, wait=True)
group.stop()
print("left", group.get_current_pose().pose)

joint_goal = group.get_current_joint_values()
joint_goal[0] = -pi
joint_goal[1] = 0.0
joint_goal[2] = -pi

group.go(joint_goal, wait=True)
group.stop()
print("down", group.get_current_pose().pose)

joint_goal = group.get_current_joint_values()
joint_goal[0] = pi/2
joint_goal[1] = 0.0
joint_goal[2] = -pi/2

group.go(joint_goal, wait=True)
group.stop()
print("right", group.get_current_pose().pose)

joint_goal = group.get_current_joint_values()
joint_goal[0] = pi
joint_goal[1] = 0.0
joint_goal[2] = -pi

group.go(joint_goal, wait=True)
group.stop()
print("down", group.get_current_pose().pose)


joint_goal = group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = 0.0
joint_goal[2] = 0

group.go(joint_goal, wait=True)
group.stop()

group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
