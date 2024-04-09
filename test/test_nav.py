#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy

from tamlib.utils import Logger
from geometry_msgs.msg import Pose2D
from sigverse_hsrb_nav import HSRBNavigation


class TestNavigation(Logger):
    def __init__(self):
        Logger.__init__(self)
        self.nav_goal = Pose2D(2.0, 1.0, 1.57)
        self.hsrbnav = HSRBNavigation()

    def test_nav(self):
        self.hsrbnav.navigation(self.nav_goal)


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param("~loop_rate", 30)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = TestNavigation()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.run()
        except rospy.exceptions.ROSException:
            rospy.logerr(f"[{rospy.get_name()}]: FAILURE")

        loop_wait.sleep() 


if __name__ == "__main__":
    main()
