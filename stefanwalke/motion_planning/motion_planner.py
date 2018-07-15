import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
	all_equal = True
	if type(goal) is list:
		for index in range(len(goal)):
			if abs(actual[index] - goal[index]) > tolerance:
				return False
	elif type(goal) is geometry_msgs.msg.PoseStamped:
		return all_close(goal.pose, actual.pose, tolerance)

	elif type(goal) is geometry_msgs.msg.Pose:
		return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

	return True

class MotionPlanner(object):
	def __init__(self):
		super(MotionPlanner, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('motion_planner', anonymous=True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		group_name = "left_arm"
		group = moveit_commander.MoveGroupCommander(group_name)

		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

		planning_frame = group.get_planning_frame()
		print "============ Reference frame: %s" % planning_frame

		eef_link = group.get_end_effector_link()
		print "============ End effector: %s" % eef_link

		group_names = robot.get_group_names()
		print "============ Robot Groups:", robot.get_group_names()

		print "============ Printing robot state"
		print robot.get_current_state()
	 	print group.get_current_joint_values()
	 	print group.get_current_pose()

		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names

	def go_to_pose_goal(self):

		group = self.group
		robot = self.robot

		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = 0.3
		pose_goal.position.y = 0.3
		pose_goal.position.z = 0.3
		group.set_pose_target(pose_goal)
		# group.set_planning_time(10)
		# group.set_planner_id("KPIECEkConfigDefault")

		plan = group.plan()
		print plan
		group.execute(plan, wait=True)
		
		group.stop()

		group.clear_pose_targets()

		current_pose = self.group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)

def main():
	try:
		tutorial = MotionPlanner()
		raw_input()
		tutorial.go_to_pose_goal()
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()