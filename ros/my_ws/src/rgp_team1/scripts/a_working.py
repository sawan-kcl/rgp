#!/usr/bin/env python

import tf
import sys
import rospy
import actionlib
import franka_gripper.msg
from a_dir_new import *
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_srvs.srv import Empty
import copy

panda_x = -0.1
panda_z = 0.615

obj_lst_top = [bBottle1,rcan2, gcan3, 
bBottle2, rBottle1, gcan2, yBottle4, 
ycan4, bBottle3, rcan1, yBottle3, ycan3,
gcan4, pouch1, pouch7, pouch1, pouch6, pouch2, 
pouch3, ycan2, ycan1, yBottle2, rcan3, 
pouch5, pouch4, rBottle2, yBottle1, gcan1]

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

  elif type(goal) is PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('rgp_pick_place_node',anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_max_velocity_scaling_factor(1)
    group.set_max_acceleration_scaling_factor(1)

    self.client1 = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
    self.client2 = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    planning_frame = group.get_planning_frame()
    # print("============ Reference frame: %s" % planning_frame)

    eef_link = group.get_end_effector_link()
    # print("============ End effector: %s" % eef_link)

    group_names = robot.get_group_names()
    # print("============ Robot Groups:", robot.get_group_names())

    # print("============ Printing robot state")
    # print(robot.get_current_state())
    # print("")

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
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


  def go_to_pose_goal(self, pose_goal):

    group = self.group

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose = self.group.get_current_pose().pose

    return all_close(pose_goal, current_pose, 0.02)

  def clear_octomap(self):
    rospy.wait_for_service('/clear_octomap')
    clear_octomap_px = rospy.ServiceProxy('/clear_octomap', Empty)
    clear_octomap_px()

  def add_box(self, obj_name, pose, box_lbh, timeout=4):

    scene = self.scene
    frame = self.robot.get_planning_frame()

    box_pose = PoseStamped()
    box_pose.header.frame_id = 'world'

    box_name = 'box'
    box_pose.pose = pose
    # scene.add_box(box_name, box_pose, size=box_lbh)
    scene.add_cylinder(box_name, box_pose, height=box_lbh[2]-0.001, radius=box_lbh[1]/2)

    self.box_name=box_name
    self.clear_octomap()

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def remove_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)
    self.clear_octomap()

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def form_pose_msg(self, obj_name):
    obj_pose_Msg = Pose()

    obj_pose_Msg.position.x = obj_name.x 
    obj_pose_Msg.position.y = obj_name.y
    obj_pose_Msg.position.z = obj_name.z 

    euler = tf.transformations.euler_from_quaternion((obj_name.q1,
                                                      obj_name.q2,
                                                      obj_name.q3,
                                                      obj_name.q4))
    if obj_name.obj_t == 'bin':
      rr,pp,yy = euler[0], pi, euler[2]
    else:  
      rr,pp,yy = euler[0], euler[1], euler[2] 

    quaternion = tf.transformations.quaternion_from_euler(rr,pp,yy)                                                                                                     

    obj_pose_Msg.orientation.x = quaternion[0]
    obj_pose_Msg.orientation.y = quaternion[1]
    obj_pose_Msg.orientation.z = quaternion[2]
    obj_pose_Msg.orientation.w = quaternion[3]

    return obj_pose_Msg

  def form_pre_pose_msg(self, poseMsg):
    pre_pose = Pose()
    pre_pose.position.x = poseMsg.position.x
    pre_pose.position.y = poseMsg.position.y
    pre_pose.position.z = poseMsg.position.z + 0.5

    rr,pp,yy = 0, pi, 0                                                    
    newQuat = tf.transformations.quaternion_from_euler(rr,pp,yy)

    pre_pose.orientation.x = newQuat[0]
    pre_pose.orientation.y = newQuat[1]
    pre_pose.orientation.z = newQuat[2]
    pre_pose.orientation.w = newQuat[3]

    return pre_pose
  
  # def get_grasp_pose(self, pose, obj):
  #   g_pose = Pose()

  #   g_pose.position.x = pose.position.x
  #   g_pose.position.y = pose.position.y
  #   g_pose.position.z = pose.position.z
  #   g_pose.orientation.x = pose.orientation.x
  #   g_pose.orientation.y = pose.orientation.y
  #   g_pose.orientation.z = pose.orientation.z
  #   g_pose.orientation.w = pose.orientation.w

  #   if (any((value <= -0.5) or (value >= 0.5) for value in (obj.q1, obj.q2))):
  #     g_pose.position.z = g_pose.position.z -0.5 + obj.b + 0.02
  #     print('fallen position')
  #   else:
  #     g_pose.position.z = g_pose.position.z -0.5 + obj.h + 0.02
  #     print('upright position')
  #   return g_pose

  def open_gripper(self):
    client1 = self.client1
    client1.wait_for_server()

    goal = franka_gripper.msg.MoveGoal(width = 0.08, speed = 0.1)

    client1.send_goal(goal)
    client1.wait_for_result()

    return client1.get_result()

  def close_gripper(self):
    client2 = self.client2
    client2.wait_for_server()

    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.01
    goal.epsilon.inner = 0.09
    goal.epsilon.outer = 0.09
    goal.speed = 0.1
    goal.force = 5.0

    client2.send_goal(goal)
    client2.wait_for_result()

    return client2.get_result()

  def attach_box(self, timeout=4):
    # print(self.group_names)
    grasping_group = "panda_hand"
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)
    return self.wait_for_state_update(
        box_is_attached=True, box_is_known=False, timeout=timeout
    ) 

  def detach_box(self, timeout=4):
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)
    return self.wait_for_state_update(
        box_is_known=True, box_is_attached=False, timeout=timeout
    )

  def plan_cartesian_path(self, obj, scale=1):
    waypoints = []
    wpose = self.group.get_current_pose().pose
    if obj.state:
      reduce_h = obj.h/2
      print("upright")
    else:
      reduce_h = obj.b/2+0.1
      print("fallen")
    wpose.position.z += scale * (0.5-reduce_h-0.08)  # move (z)
    waypoints.append(copy.deepcopy(wpose))
    (plan,fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

    self.group.execute(plan, wait=True)
    # return plan, fraction

  def pick_n_place(self, obj_name):
    if obj_name.obj_t == 'can':
      bin_pose = self.form_pose_msg(GreenBin)
    else:
      bin_pose = self.form_pose_msg(BlueBin)

    main_pose = self.form_pose_msg(obj_name)
    pre_pose = self.form_pre_pose_msg(main_pose)
    # grasp_pose = self.get_grasp_pose(pre_pose, obj_name)

    l,b,h = obj_name.l, obj_name.b, obj_name.h

    self.add_box(obj_name, main_pose, (l,b,h))

    # pick
    self.open_gripper()
    try:
      self.go_to_pose_goal(pre_pose)
    except:
      raise RuntimeError("Failed to find path step 1")
    try:
      self.plan_cartesian_path(obj_name, scale = -1)
    except:
      raise RuntimeError("Failed to find path step 2")
    self.attach_box()
    self.close_gripper()
    try:
      self.plan_cartesian_path(obj_name, scale = 1)
    except:
      raise RuntimeError("Failed to find path step 3")

    # rospy.sleep(1)

    # place
    try:
      self.go_to_pose_goal(bin_pose) 
    except:
      raise RuntimeError("Failed to find path step 4")

    self.open_gripper()
    self.detach_box()
    self.remove_box()

def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()
        for i in obj_lst_top:
          tutorial.pick_n_place(i)
        # tutorial.open_gripper()
        # tutorial.detach_box()
        # tutorial.remove_box()
        # tutorial.pick_n_place(gcan2)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()