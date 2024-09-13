#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates

def callback(data):
   for i in range(len(data.name) -1):
        print("object: %s, x: %f, y: %f, z: %f" %(data.name[i+1], data.pose[i+1].position.x, data.pose[i+1].position.y, data.pose[i+1].position.z))
    
def listener():
    rospy.init_node('gazebo_obj_pose_pub', anonymous=True)

    rospy.Subscriber("/gazebo/model_states", ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
