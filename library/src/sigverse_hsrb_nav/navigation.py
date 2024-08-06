#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import tf
import math
import rospy
import tf2_ros
import actionlib
import traceback
import numpy as np
import tf2_geometry_msgs
import dynamic_reconfigure.client

from tamlib.utils import Logger

from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class HSRBNavigation(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")
        self.local_il_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer")
        self.global_il_client = dynamic_reconfigure.client.Client("/move_base/global_costmap/inflation_layer")

        self.tf_buf = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tf_buf)

        # ros interface
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move base server")

        self.rate = rospy.Rate(10)
        self.timeout = rospy.Duration(60)

    def _set_initialpose(self, x: float, y: float, yaw: float) -> None:
        """
        """
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = 0
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.pose.orientation.x = quat[0]
        pose.pose.pose.orientation.y = quat[1]
        pose.pose.pose.orientation.z = quat[2]
        pose.pose.pose.orientation.w = quat[3]

        self.initial_pose_pub.publish(pose)

    def cancel_goal(self) -> False:
        """move baseに関するアクションをすべてキャンセルする関数
        Args:
        Returns:
            bool: キャンセルに成功したかどうか
        """
        try:
            self.move_base_client.cancel_all_goals()
            self.loginfo("success: cancel all goals")
            return True
        except Exception as e:
            self.logwarn(e)
            self.logwarn("failure: cancel all goals")
            return False

    def update_local_costmap(self, radius=0.75):
        self.local_il_client.update_configuration({"inflation_radius": radius})

    def update_global_costmap(self, radius=0.75):
        self.global_il_client.update_configuration({"inflation_radius": radius})

    def navigation(self, goal_pose: Pose2D, mode="abs", goal_torelance=(0.05, 0.05)) -> bool:
        """navigationを行う関数
        Args:
            goal_pose(Pose2D): ナビゲーションの目的地を設定
            mode(str): 絶対移動か相対移動かを指定(abs or rel)
                default to abs
            goal_torelance: 目標位置との許容誤差
        Return:
            bool: ナビゲーションに成功したかどうか
        """
        try:
            start_time = rospy.Time.now()
            self.loginfo(f"navigation mode is {mode}")
            self.loginfo(f"navigation goal x={goal_pose.x}, y={goal_pose.y}, yaw={goal_pose.theta}")

            goal_msg = MoveBaseGoal()

            if mode == 'abs':
                goal_msg.target_pose.header.frame_id = 'map'
            elif mode == 'rel':
                goal_msg.target_pose.header.frame_id = 'base_footprint'
            # 不明なモードが選択された場合は絶対移動に変更
            else:
                self.logwarn("mode must be set abs or rel.")
                goal_msg.target_pose.header.frame_id = 'map'

            goal_msg.target_pose.header.stamp = rospy.Time.now()
            goal_msg.target_pose.pose.position.x = goal_pose.x
            goal_msg.target_pose.pose.position.y = goal_pose.y

            q = tf.transformations.quaternion_from_euler(0, 0, goal_pose.theta)
            goal_msg.target_pose.pose.orientation.x = q[0]
            goal_msg.target_pose.pose.orientation.y = q[1]
            goal_msg.target_pose.pose.orientation.z = q[2]
            goal_msg.target_pose.pose.orientation.w = q[3]

            self.move_base_client.send_goal(goal_msg)
            while (rospy.Time.now() - start_time) < self.timeout:
                action_state = self.move_base_client.get_state()

                if action_state == actionlib.GoalStatus.SUCCEEDED:
                    self.loginfo("reached to goal position")
                    return True
                elif action_state == actionlib.GoalStatus.ABORTED:
                    self.loginfo("aborted to reach goal")
                    return False

                elif action_state == actionlib.GoalStatus.PREEMPTED:
                    self.loginfo("preempted to reach goal")
                    return False

                trans = self.tf_buf.lookup_transform('map', 'base_footprint', rospy.Time.now(), rospy.Duration(5.))
                current_x, current_y = trans.transform.translation.x, trans.transform.translation.y
                self.logdebug(trans)

                current_dis = math.sqrt((goal_pose.x - current_x) ** 2 + (goal_pose.y - current_y) ** 2)
                if current_dis < goal_torelance[0]:
                    self.move_base_client.cancel_goal()
                    self.loginfo('get new goal position')
                    return True

                self.rate.sleep()

            self.move_base_client.cancel_goal()
            self.loginfo('timeout to reach goal')
            return False

        except Exception as e:
            self.logwarn(e)
            self.loginfo("navigation failed")
            return False
