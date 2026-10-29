#! /usr/bin/env python

import time
import unittest
import math

import rosunit
import rostest
from tf.transformations import euler_from_quaternion
import actionlib
import rospy

from tortoisebot_waypoints.msg import WaypointActionGoal, WaypointActionFeedback, WaypointActionResult, WaypointActionAction
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry



PKG = 'tortoisebot_waypoints'
NAME = 'waypoints_test'


class TestWaypoints(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_waypoints')

    def setUp(self):
        self.waypoint_action_client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)

        rospy.loginfo("Waiting for tortoisebot_as action server...")
        self.assertTrue(self.waypoint_action_client.wait_for_server(rospy.Duration(5.0)), "tortoisebot_as action server not available.")

        self.active = False
        self.feedback_list = []

    def tearDown(self):
        self.waypoint_action_client.cancel_all_goals()

    def test_waypoints_pos(self):
        before_odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
        x = before_odom_msg.pose.pose.position.x
        y = before_odom_msg.pose.pose.position.y
        # Depending on where robot is, different goal positions should be used since even the correct waypoint action server isn't always successful
        # if -0.1 < x and x < 0.2 and -0.2 < y and y < 0.3:
        #     goal_position = Point(x=0.0, y=0.0, z=0.0)
        # elif x > 0.0 and y > 0.0:
        #     goal_position = Point(x=0.6, y=0.2, z=0.0)
        # elif x > 0.0 and y <= 0.0:
        #     goal_position = Point(x=0.3, y=-0.2, z=0.0)
        # elif x <= 0.0 and y <= -0.3:
        #     goal_position = Point(x=-0.3, y=-0.4, z=0.0)
        # elif x <= 0.0 and y <= 0.0:
        #     goal_position = Point(x=-0.3, y=-0.2, z=0.0)
        # elif x <= 0.0 and y > 0.0:
        #     goal_position = Point(x=-0.3, y=0.2, z=0.0)
        # goal_position = Point(x=before_odom_msg.pose.pose.position.x-0.07, y=before_odom_msg.pose.pose.position.y, z=before_odom_msg.pose.pose.position.z)
        goal_position = Point(x=0.0, y=0.0, z=0.0)
        waypoint_goal = WaypointActionGoal(position=goal_position)

        self.waypoint_action_client.send_goal(waypoint_goal,
                    done_cb=self.done_cb,
                    active_cb=self.active_cb,
                    feedback_cb=self.feedback_cb)
        
        finished = self.waypoint_action_client.wait_for_result(rospy.Duration(45.0))
        self.assertTrue(finished, "Took too long for action server to give result")

    
        self.assertEqual(self.waypoint_action_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(self.waypoint_action_client.get_result().success)

        self.assertTrue(self.active)

        after_odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
        self.assertAlmostEqual(after_odom_msg.pose.pose.position.x, goal_position.x, delta=0.05)
        self.assertAlmostEqual(after_odom_msg.pose.pose.position.y, goal_position.y, delta=0.05)

    def test_waypoints_yaw(self):
        before_odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
        x = before_odom_msg.pose.pose.position.x
        y = before_odom_msg.pose.pose.position.y
        # goal_position = Point(x=-0.1, y=-0.1, z=0.0)
        # if -0.1 < x and x < 0.2 and -0.2 < y and y < 0.3:
        #     goal_position = Point(x=0.0, y=0.1, z=0.0)
        # elif x > 0.0 and y > 0.0:
        #     goal_position = Point(x=0.6, y=0.1, z=0.0)
        # elif x > 0.0 and y <= 0.0:
        #     goal_position = Point(x=0.3, y=-0.1, z=0.0)
        # elif x <= 0.0 and y <= -0.3:
        #     goal_position = Point(x=-0.3, y=-0.3, z=0.0)
        # elif x <= 0.0 and y <= 0.0:
        #     goal_position = Point(x=-0.3, y=-0.1, z=0.0)
        # elif x <= 0.0 and y > 0.0:
        #     goal_position = Point(x=-0.3, y=0.1, z=0.0)
        # goal_position = Point(x=before_odom_msg.pose.pose.position.x+0.07, y=before_odom_msg.pose.pose.position.y, z=before_odom_msg.pose.pose.position.z)
        goal_position = Point(x=0.0, y=0.1, z=0.0)
        waypoint_goal = WaypointActionGoal(position=goal_position)
        desired_yaw = math.atan2(goal_position.y - before_odom_msg.pose.pose.position.y, goal_position.x - before_odom_msg.pose.pose.position.x)
        

        self.waypoint_action_client.send_goal(waypoint_goal,
                    done_cb=self.done_cb,
                    active_cb=self.active_cb,
                    feedback_cb=self.feedback_cb)
        
        finished = self.waypoint_action_client.wait_for_result(rospy.Duration(45.0))
        self.assertTrue(finished, "Took too long for action server to give result")
    
        after_odom_msg = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
        quat = after_odom_msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        self.assertEqual(self.waypoint_action_client.get_state(), GoalStatus.SUCCEEDED)
        self.assertTrue(self.waypoint_action_client.get_result().success)

        self.assertTrue(self.active)
        # self.assertGreater(len(self.feedback_list), 0, "No feedback messages in feedback list")

        # end_feedback = self.feedback_list[-1]
        # self.assertAlmostEqual(end_feedback.position.x, goal_position.x, places=1)
        # self.assertAlmostEqual(end_feedback.position.y, goal_position.y, places=1)

        self.assertAlmostEqual(yaw, desired_yaw, delta=0.6)

    def active_cb(self):
        self.active = True

    def feedback_cb(self, feedback):
        self.feedback_list.append(feedback)

    def done_cb(self, state, result):
        rospy.loginfo("Goal done: %s", GoalStatus.to_string(state))
        

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypoints)