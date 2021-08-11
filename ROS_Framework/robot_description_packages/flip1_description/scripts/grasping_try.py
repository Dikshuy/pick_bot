#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
robot = moveit_commander.RobotCommander()

manipulator_group=moveit_commander.MoveGroupCommander("manipulator")

gripper_group=moveit_commander.MoveGroupCommander("gripper")

manipulator_group.set_named_target("place1")
plan1 = manipulator_group.go()



manipulator_group.set_named_target("place2")
plan3 = manipulator_group.go()

gripper_group.set_named_target("place_gripper_open")
plan2 = gripper_group.go()

rospy.sleep(1)
moveit_commander.roscpp_shutdown()
