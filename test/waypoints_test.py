#! /usr/bin/env python

import time
import actionlib
import rospy
import rosunit
import unittest
import rostest

from tortoisebot_waypoints.msg import WaypointActionGoal, WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point

PKG = 'tortoisebot_waypoints'
NAME = 'waypoints_test'


class TestWaypoints(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_waypoints')

        self.waypoint_action_client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

        rospy.loginfo("Waiting for tortoisebot_as action server...")
        self.assertTrue(self.waypoint_action_client.wait_for_server(rospy.Duration(5.0)), "tortoisebot_as action server not available. Exiting...")

        self.active = False
        self.feedback_list = []

    def test_waypoints(self):
            
        waypoint_goal = WaypointActionGoal(position=Point(x=0.0, y=0.0, z=0.0))

        self.waypoint_action_client.send_goal(waypoint_goal,
                    done_cb=self.done_cb,
                    active_cb=self.active_cb,
                    feedback_cb=self.feedback_cb)
        
        finished = self.waypoint_action_client.wait_for_result(rospy.Duration(30.0))
        self.assertTrue(finished, "Took too long for action server to give result")
    
        self.assertEqual(self.waypoint_action_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(self.waypoint_action_client.get_result().success)

        self.assertTrue(self.active)
        self.assertGreater(len(self.feedback_list), 0, "No feedback messages in feedback list")

        end_feedback = self.feedback_list[-1]
        self.assertAlmostEqual(end_feedback.position.x, 0.0, places=1)
        self.assertAlmostEqual(end_feedback.position.y, 0.0, places=1)

    def active_cb(self):
        self.active = True

    def feedback_cb(self, feedback):
        self.feedback_list.append(feedback)

    def done_cb(self, state, result):
        rospy.loginfo("Goal done: %s", GoalStatus.to_string(state))
        


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypoints)