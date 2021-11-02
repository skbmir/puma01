#!/usr/bin/env python

import rospy
import time
# import thread
import actionlib

from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory
# from robot_movement_interface.msg import *
from sensor_msgs.msg	 import JointState
from std_msgs.msg	 import Float64MultiArray

# conf_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# lock = thread.allocate_lock()
# joint_states = JointState()

conf_blending = 0.02 # M

# ------------------------------------------------------------------------
# Callback function executed after the publication of the current robot position
# ------------------------------------------------------------------------
# def joint_states_callback(msg):
# 	with lock:
# 		global joint_states
# 		joint_states = msg

class MoveItTrajectoryExecuter(object):
	
	# _feedback = FollowJointTrajectoryFeedback()
	# _result = FollowJointTrajectoryResult()

	# ---------------------------------------------------------------------------------------
	# Rearranges the point path following the name convention joint_0, ... joint_6
	# Warning: This function has side effects
	# ---------------------------------------------------------------------------------------
	# def rearrange(self, joint_trajectory):

	# 	mapping = [joint_trajectory.joint_names.index(j) for j in conf_joint_names]

	# 	for point in joint_trajectory.points:

	# 		temp_positions = []
	# 		temp_velocities = []
	# 		temp_accelerations = []
	# 		temp_effort = []

	# 		for i in range(len(point.positions)):
	# 			temp_positions.append(point.positions[mapping[i]])
	# 		for i in range(len(point.velocities)):
	# 			temp_velocities.append(point.velocities[mapping[i]])
	# 		for i in range(len(point.accelerations)):
	# 			temp_accelerations.append(point.accelerations[mapping[i]])
	# 		for i in range(len(point.effort)):
	# 			temp_effort.append(point.effort[mapping[i]])

	# 		point.positions = temp_positions
	# 		point.velocities = temp_velocities
	# 		point.accelerations = temp_accelerations
	# 		point.effort = temp_effort

	# 	joint_trajectory.joint_names = conf_joint_names

	# Action initialisation
	def __init__(self, name):
		self.cmd_pub = rospy.Publisher('/puma01/cmd_trajectory_point',Float64MultiArray, queue_size=1)
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

	# Action callback
	def execute_cb(self, goal):

		# It is required to rearrange the arrays because MoveIt doesn't guarantee orden preservation
		# self.rearrange(goal.trajectory)

		# A trajectory needs at least 2 points		
		if len(goal.trajectory.joint_trajectory.points) < 2:
			return

		rospy.loginfo("Execution begins!")

		#time_start = rospy.Time.from_sec(time.time())

		# ------------- Send command list

		trajectory_point = Float64MultiArray()

		last_time = 0

		for goal_point in goal.trajectory.joint_trajectory.points:

			rospy.sleep(goal_point.time_from_start.to_sec()  - last_time)

			trajectory_point.data.append(goal_point.positions)
			trajectory_point.data.append(goal_point.velocities)
			trajectory_point.data.append(goal_point.accelerations)

			self.cmd_pub.publish(trajectory_point)

			trajectory_point.data.clear()

			last_time = goal_point.time_from_start.to_sec() 

			rospy.loginfo("Point sent...")

		# ------------- Wait until the termination while providing feedback

		# last_point = goal.trajectory.points[0]
		# for point in goal.trajectory.points[1:]:
		# 	# Wait	
		# 	rospy.sleep(point.time_from_start - last_point.time_from_start)
		# 	# Trajectory abort!
		# 	# To abort the current movement, it is possible to send an empty trajectory
		# 	if self._as.is_preempt_requested():
		# 		trajectory_2 = CommandList()
		# 		trajectory_2.replace_previous_commands = True
		# 		self.cmd_pub.publish(trajectory_2)
		# 		self._as.set_preempted()
		# 		return
		# 	# ---------------------------------------
		# 	# Feedback
		# 	self._feedback.joint_names = goal.trajectory.joint_names
		# 	self._feedback.desired = point
		# 	with lock:
		# 		self._feedback.actual.positions = joint_states.position
		# 		self._feedback.actual.velocities = joint_states.velocity
		# 		self._feedback.actual.time_from_start = rospy.Time.from_sec(time.time()) - time_start
		# 	self._as.publish_feedback(self._feedback)
		# 	# ---------------------------------------
		# 	last_point = point

		# # ---------------------------------------
		# # Result
		# self._result.error_code = 0
		# self._as.set_succeeded(self._result)
		# # ---------------------------------------

if __name__ == '__main__':
	rospy.init_node('puma01_trajectory_executer')
	# rospy.Subscriber('/puma01/joint_states', JointState, joint_states_callback)
	MoveItTrajectoryExecuter(rospy.get_name())
	rospy.loginfo("Trajectory executer started.")
	rospy.spin()
