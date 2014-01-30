#!/usr/bin/env python
# Software License Agreement (BSD) 
#
# Author    Alex Bencz <abencz@clearpathrobotics.com>
# Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('ur_driver')
import rospy

import actionlib

from functools import partial
from math import pi

from std_msgs.msg import Float64
from control_msgs.msg import *
from trajectory_msgs.msg import *


class UR5Action(object):
    _feedback = FollowJointTrajectoryActionFeedback()
    _result = FollowJointTrajectoryActionResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 
                                                FollowJointTrajectoryAction, 
                                                execute_cb=self.execute_cb, 
                                                auto_start=False)
        self._as.start()

        self._publishers = [
            rospy.Publisher('shoulder_pan_position_controller/command', Float64),
            rospy.Publisher('shoulder_lift_position_controller/command', Float64),
            rospy.Publisher('elbow_position_controller/command', Float64),
            rospy.Publisher('wrist_1_position_controller/command', Float64),
            rospy.Publisher('wrist_2_position_controller/command', Float64),
            rospy.Publisher('wrist_3_position_controller/command', Float64)]

        def state_callback(variables, joint, data):
            variables[joint] = data.process_value

        self._state = {'shoulder_pan': 0,
                'shoulder_lift': 0,
                'elbow': 0,
                'wrist_1': 0,
                'wrist_2': 0,
                'wrist_3': 0}

        rospy.Subscriber("shoulder_pan_position_controller/state", JointControllerState,
                partial(state_callback, self._state, 'shoulder_pan'))
        rospy.Subscriber("shoulder_lift_position_controller/state", JointControllerState,
                partial(state_callback, self._state, 'shoulder_lift'))
        rospy.Subscriber("elbow_position_controller/state", JointControllerState,
                partial(state_callback, self._state, 'elbow'))
        rospy.Subscriber("wrist_1_position_controller/state", JointControllerState,
                partial(state_callback, self._state, 'wrist_1'))
        rospy.Subscriber("wrist_2_position_controller/state", JointControllerState,
                partial(state_callback, self._state, 'wrist_2'))
        rospy.Subscriber("wrist_3_position_controller/state", JointControllerState,
                partial(state_callback, self._state, 'wrist_3'))

    def command_arm(self, *args):
        if len(args) < 6:
            # not enough joint positions to move arm
            return

        for pub, value in zip(self._publishers, args):
            pub.publish(value)

    def close_enough(self, *args):
        tau = pi * 2

        for target, actual in zip(args, self._state.values()):
            while target < 0:
                target += tau
            while target > tau:
                target -= tau
            while actual < 0:
                actual += tau
            while actual > tau:
                actual -= tau
            if abs(target - actual) < 0.1:
                return True
            if abs(abs(target - actual) - tau) < 0.1:
                return True

        return False

    def execute_cb(self, goal):
        timeout = 30 #seconds
        success = True
        trajectory = goal.trajectory
        points = trajectory.points

        for p in points:
            positions = p.positions
            self.command_arm(*positions)
            
            r = rospy.Rate(0.1)
            start_time = rospy.get_time()
            while not self.close_enough(*positions):
                r.sleep()
                if (rospy.get_time() - start_time > timeout):
                    success = False
                    break

        result = FollowJointTrajectoryResult()
        if success:
            self._as.set_succeeded(result)

      
if __name__ == '__main__':
    rospy.init_node('ur5_action')
    UR5Action('ur5_action')
    rospy.spin()
