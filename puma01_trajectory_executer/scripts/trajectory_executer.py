#!/usr/bin/env python

import rospy
import time

from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg	 import Float64MultiArray, MultiArrayDimension

class MoveItPlannedTrajectoryExecuter(object):

	def __init__(self, name):
	
	# set the topic to send trajectory commands to
		self.cmd_pub = rospy.Publisher('/puma01/computed_torque_controller/command',Float64MultiArray, queue_size=1)
		
		rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, self.planned_trajectory_receive)

	def planned_trajectory_receive(self, goal):

		traj_len = len(goal.trajectory[0].joint_trajectory.points)

		if traj_len < 2:
			return

		rospy.loginfo("Planned trajectory received!")

		trajectory_point = Float64MultiArray()

		last_time = 0

		traj_num = 0

		for goal_point in goal.trajectory[0].joint_trajectory.points:

			rospy.sleep(goal_point.time_from_start.to_sec()  - last_time)

			trajectory_point.data = [*goal_point.positions, *goal_point.velocities, *goal_point.accelerations]

			traj_num = traj_num + 1

			# trajectory_point.data[1:6] = goal_point.positions
			# trajectory_point.data[7:12] = goal_point.velocities
			# trajectory_point.data[13:18] = goal_point.accelerations

			# dim = []
			# dim.append(MultiArrayDimension("point",18,1))
			# trajectory_point.layout.dim = dim

			#rospy.loginfo(type(trajectory_point.data[0]))

			if traj_num==traj_len:
				trajectory_point.data = [*goal_point.positions, *goal_point.velocities, 0, 0, 0, 0, 0, 0]

			self.cmd_pub.publish(trajectory_point)

			trajectory_point.data = []
			last_time = goal_point.time_from_start.to_sec() 

			#rospy.loginfo("Point set: ",str(goal_point.header.seq))

if __name__ == '__main__':
	rospy.init_node('puma01_trajectory_executer')
	# rospy.Subscriber('/puma01/joint_states', JointState, joint_states_callback)
	MoveItPlannedTrajectoryExecuter(rospy.get_name())
	rospy.loginfo("Trajectory executer started.")
	rospy.spin()
