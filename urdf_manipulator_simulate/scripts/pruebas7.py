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


group.set_planning_time(600.0)

current_pose = group.get_current_pose().pose
print(current_pose)

pose_goal = [0.0, 0.0, 0.0, 0.0]

# Pose Position
pose_goal[0] = -0.392204879921
pose_goal[1] = 0.588028204094
pose_goal[2] = -0.39345410069
pose_goal[3] = 0.587870763013

print(pose_goal)
group.set_goal_orientation_tolerance(0.1745)

group.set_orientation_target(pose_goal)
plan = group.plan()

if plan.joint_trajectory.points:  # True if trajectory contains points
    move_success = group.execute(plan)
else:
    rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

group.stop()
group.clear_pose_targets()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
