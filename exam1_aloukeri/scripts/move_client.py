#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import actionlib
from exam1_aloukeri.msg import MoveRobotAction, MoveRobotGoal


def feedback_cb(feedback):
    rospy.loginfo("Travel Time: %.2f s", feedback.travel_time)


def main():
    rospy.init_node('move_robot_client')

    # Παράμετροι από parameter server 
    goal_x   = rospy.get_param('/exam1/goal_x',    -1.7)
    goal_y   = rospy.get_param('/exam1/goal_y',     1.07)
    max_time = rospy.get_param('/exam1/max_time', 120.0)

    client = actionlib.SimpleActionClient('move_robot', MoveRobotAction)
    rospy.loginfo("Client- Wait for Action Server...")
    client.wait_for_server()
    rospy.loginfo("Server found!")

    # Δημιουργία και αποστολή goal
    goal   = MoveRobotGoal()
    goal.x = goal_x
    goal.y = goal_y

    rospy.loginfo("[Client] Αποστολή goal → (%.2f, %.2f)", goal_x, goal_y)
    client.send_goal(goal, feedback_cb=feedback_cb)

    # Αναμονή αποτελέσματος
    finished = client.wait_for_result(rospy.Duration(max_time + 15.0))

    if finished:
        result = client.get_result()
        if result.success:
            rospy.loginfo("Success! Duration: %.2f s", result.travel_time)
        else:
            rospy.logwarn("Failed with duration: %.2f s",
                          result.travel_time)
    else:
        rospy.logerr("Abort goal.")
        client.cancel_goal()


if __name__ == '__main__':
    main()