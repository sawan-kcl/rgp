#!/usr/bin/env python

import tf
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# import copy


panda_x = -0.1
panda_z = 0.615

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False


  def go_to_pose_goal(self):

    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.415 - panda_x
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.8150 - panda_z
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    
    group.stop()

    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def add_box(self, timeout=4):

    # box_name = self.box_name
    scene = self.scene
    frame = self.robot.get_planning_frame()

    # rospy.sleep(2)
    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame
    box_pose.pose.position.x = 0.22165
    box_pose.pose.position.y = -0.021974
    box_pose.pose.position.z = 0.547836

    quaternion = tf.transformations.quaternion_from_euler(1.569748, -1.472617, 0.010242)
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]

    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def remove_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()

        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        # tutorial.go_to_pose_goal()

        input("============ Press `Enter` to add a box to the planning scene ...")
        tutorial.add_box()

        input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
        tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
