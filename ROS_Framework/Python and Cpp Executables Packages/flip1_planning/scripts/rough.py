#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    planning_frame = manipulator_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = manipulator_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.manipulator_group = manipulator_group
    self.gripper_group = gripper_group

    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def before_adding_box(self):
    manipulator_group = self.manipulator_group
    gripper_group = self.gripper_group
    manipulator_group.set_named_target("start")
    plan1 = manipulator_group.go()

    gripper_group.set_named_target("gripper_turn1")
    plan2 = gripper_group.go()

    manipulator_group.set_named_target("come_down")
    plan1 = manipulator_group.go()


  def before_adding_box_GRIPPER(self):
    manipulator_group = self.manipulator_group
    gripper_group = self.gripper_group
    print "GRIPPER TO BE CLOSED"
    gripper_group.set_named_target("gripper_close")
    plan2 = gripper_group.go()


  def after_attching_box(self):
      manipulator_group = self.manipulator_group
      gripper_group = self.gripper_group

      #gripper_group.set_named_target("gripper_turn2")
      #plan2 = gripper_group.go()

      manipulator_group.set_named_target("place1")
      plan1 = manipulator_group.go()

      manipulator_group.set_named_target("place2")
      plan1 = manipulator_group.go()

  def placing_box(self):
      manipulator_group = self.manipulator_group
      gripper_group = self.gripper_group
      gripper_group.set_named_target("place_gripper_open")
      plan2 = gripper_group.go()

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

      # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL


  def box_before_scene(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "table1"

    ####TEJAL-----https://answers.ros.org/question/254650/move_group_interface_tutorial-orientation-vs-position/
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0 # slightly above the end effector
    box_pose.pose.position.z = 0.325
    box_name = "box1"
    scene.add_box(box_name, box_pose, size=(0.15, 0.08, 0.15))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def add_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link_gripper"
    q = quaternion_from_euler(0, 0, 1.5707)
    print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
    box_pose.pose.orientation.x = q[0]
    box_pose.pose.orientation.y = q[1]
    box_pose.pose.orientation.z = q[2]
    box_pose.pose.orientation.w = q[3]
    ####TEJAL-----https://answers.ros.org/question/254650/move_group_interface_tutorial-orientation-vs-position/
    box_pose.pose.position.y = 0.3792 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.15, 0.1, 0.15))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):

    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    grasping_group = 'gripper'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)

    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the demo of robot's PICK and PLACE"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Setting up the moveit_commander ==========="
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "==========Intial Scene=========="
    raw_input()
    tutorial.box_before_scene()

    print "=========Approaching towards Object to be pickedup========="

    tutorial.before_adding_box()


    print "============Trying to grasp the object============"
    tutorial.before_adding_box_GRIPPER()
    tutorial.remove_box()

    tutorial.add_box()

    print "============Grasping the object============="

    tutorial.attach_box()

    print "==========Moving towards another table to place the object==========="

    tutorial.after_attching_box()

    print "============Trying to place object============="

    tutorial.detach_box()

    print "===========Retreating after placing the object=========="

    tutorial.placing_box()


    print "============Demo ends here==========="
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
