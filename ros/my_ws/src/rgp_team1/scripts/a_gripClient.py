#! /usr/bin/env python

import rospy
import actionlib
import franka_gripper.msg

# def myAction_client():
#     client1 = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
#     client1.wait_for_server()

#     goal = franka_gripper.msg.MoveGoal(width = 0.08, speed = 0.1)

#     client1.send_goal(goal)
#     client1.wait_for_result()

#     return client1.get_result()  


def myAction_client():
    client1 = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)
    client1.wait_for_server()

    goal = franka_gripper.msg.GraspGoal()
    goal.width = 0.01
    goal.epsilon.inner = 0.09
    goal.epsilon.outer = 0.09
    goal.speed = 0.1
    goal.force = 5.0

    client1.send_goal(goal)
    client1.wait_for_result()

    return client1.get_result() 

if __name__ == '__main__':
    try:
        rospy.init_node('gripAction_test')
        result = myAction_client()
        print("Result:", result.success)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")