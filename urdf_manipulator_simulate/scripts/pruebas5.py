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

pose_goal = geometry_msgs.msg.Pose()

# Pose Position
x_coordinate = 0.445491651633
y_coordinate = -0.311112398986
z_coordinate = 0.517029078873
pose_goal.position.x = x_coordinate
pose_goal.position.y = y_coordinate
pose_goal.position.z = z_coordinate

pose_goal.orientation = current_pose.orientation

print(pose_goal)
print(group.get_goal_tolerance())
group.set_goal_position_tolerance(0.001)
group.set_goal_orientation_tolerance(0.1745)
print(group.get_goal_tolerance())
group.set_pose_target(pose_goal)
plan = group.plan()

if plan.joint_trajectory.points:  # True if trajectory contains points
    move_success = group.execute(plan)
else:
    rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

group.stop()
group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
