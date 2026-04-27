#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import actionlib
import math
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
from exam1_aloukeri.msg import MoveRobotAction, MoveRobotFeedback, MoveRobotResult


class MoveRobotServer(object):

    def __init__(self):
        self.current_x = 0.0
        self.current_y = 0.0

        self.enable_pub = rospy.Publisher('/line_follower/enable', Bool,  queue_size=1,latch=True)
        self.cmd_pub    = rospy.Publisher('/cmd_vel',               Twist, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb)

        self.server = actionlib.SimpleActionServer(
            'move_robot', MoveRobotAction,
            execute_cb=self.execute_cb, auto_start=False)
        self.server.start()
        rospy.loginfo("Server- MoveRobot Action Server initialized.")

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index('turtlebot3_waffle_pi')
            self.current_x = msg.pose[idx].position.x
            self.current_y = msg.pose[idx].position.y
        except ValueError:
            pass

    def execute_cb(self, goal):
        feedback = MoveRobotFeedback()
        result   = MoveRobotResult()

        goal_x   = goal.x
        goal_y   = goal.y
        max_time = rospy.get_param('/exam1/max_time', 120.0)

        rospy.loginfo("New goal: (%.2f, %.2f) | max_time=%.1f s",
                      goal_x, goal_y, max_time)

        # Ενεργοποίηση line_follower
        rospy.sleep(0.5)
        self.enable_pub.publish(Bool(True))

        start_time = rospy.get_time()
        rate       = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - start_time

            # ακύρωση από client
            if self.server.is_preempt_requested():
                rospy.loginfo("[Server] Goal ακυρώθηκε.")
                self.stop_all()
                self.server.set_preempted()
                return

            # Timeout -> abort
            if elapsed > max_time:
                rospy.logwarn("[Server] TIMEOUT (%.1f s) → abort.", elapsed)
                self.stop_all()
                result.travel_time = elapsed
                result.success     = False
                self.server.set_aborted(result)
                return

            # Goal reached: eucl dist < 0.2
            dist = math.sqrt((self.current_x - goal_x)**2 +
                             (self.current_y - goal_y)**2)
            if dist < 0.2:
                rospy.loginfo("Got there in time! dist=%.3f | time=%.2f s",
                              dist, elapsed)
                self.stop_all()
                result.travel_time = elapsed
                result.success     = True
                self.server.set_succeeded(result)
                return

            # Feedback: τρέχων χρόνος
            feedback.travel_time = elapsed
            self.server.publish_feedback(feedback)

            rate.sleep()

    def stop_all(self):
        self.enable_pub.publish(Bool(False))
        rospy.sleep(0.1)
        self.cmd_pub.publish(Twist())


if __name__ == '__main__':
    rospy.init_node('move_robot_server')
    MoveRobotServer()
    rospy.spin()