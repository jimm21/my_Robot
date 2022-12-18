#! /usr/bin/env python
import sys
from math import pi
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('trajectory_control', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("robot_arm")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


group.set_planning_time(10.0)

current_pose = group.get_current_pose().pose
print(current_pose)

pose_goal = [0.0, 0.0, 0.0]

# Pose Position
x_coordinate = 0.445491651633
y_coordinate = -0.001
z_coordinate = 0.8
pose_goal[0] = x_coordinate
pose_goal[1] = y_coordinate
pose_goal[2] = z_coordinate

print(pose_goal)
print(group.get_goal_tolerance())
group.set_goal_position_tolerance(0.001)
group.set_goal_orientation_tolerance(0.1745)
print(group.get_goal_tolerance())
group.set_position_target(pose_goal)
plan = group.plan()

if plan.joint_trajectory.points:  # True if trajectory contains points
    move_success = group.execute(plan)
else:
    rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

group.stop()
group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
