#!/usr/bin/env python

import rospy,tf

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

# th means the orientation to the x axis and is related to pi
def move(client, x, y, th):
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    # Waits until the action server has started up and started listening for goals.
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # set the goal pos
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # convert radian to the 3D pose
    quat = tf.transformations.quaternion_from_euler(0, 0, th)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        # return client.get_result()
        pass

# def readTxt(filename):
#     with open(filename,'r') as f:
#         # SKIP THE FIRST LINE
#         next(f)
#         lines = f.readlines()
#         for line in lines:
#             data = line.split()
#             x = data[0]
#             y = data[1]
#             radian = data[2]
#     f.close()
#     return x,y,radian

def movebase_client():
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    filename = 'map.txt'
    # reading the x,y,radians data from a customized file
    with open(filename,'r') as f:
        # SKIP THE FIRST LINE
        next(f)
        lines = f.readlines()
        for line in lines:
            data = line.split()
            x = data[0]
            y = data[1]
            radian = data[2]
            # Move robot by sending x,y,radians
            move(client, x, y, radian)
    f.close()



if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
