#!/usr/bin/env python3
"""
ROS subscribers for robot state and control data.
"""
import os
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
from math import degrees
from Attachment-Model.msg import Action, RobotPub, Emotion


class OdomSubscriber:
    """Subscribe to odometry data and track robot position/orientation."""
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/sensors/odom"
        self.subscriber = rospy.Subscriber(topic, Odometry, self._callback)

    def _callback(self, msg):
        orientation = msg.pose.pose.orientation
        position = msg.pose.pose.position
        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz'
        )
        self.yaw = round(degrees(yaw) + (360 if yaw < 0 else 0), 4)
        self.posx = round(position.x, 4)
        self.posy = round(position.y, 4)


class RangeSubscriber:
    """Subscribe to sonar range data."""
    
    def __init__(self):
        self.range = 0.0
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/sensors/sonar"
        self.subscriber = rospy.Subscriber(topic, Range, self._callback)

    def _callback(self, msg):
        self.range = msg.range


class ActionSubscriber:
    """Subscribe to central controller action commands."""
    
    def __init__(self):
        self.parent = 0
        self.child = 0
        self.emotional_distance = 0
        self.physical_distance = 0
        self.child_need = 0
        self.parent_need = 0
        self.subscriber = rospy.Subscriber("/central_controller", Action, self._callback)

    def _callback(self, msg):
        self.parent = msg.parent
        self.child = msg.child
        self.emotional_distance = msg.emotional_distance
        self.physical_distance = msg.physical_distance
        self.child_need = msg.child_need
        self.parent_need = msg.parent_need


class RobotStateSubscriber:
    """Subscribe to robot state (child or parent)."""
    
    def __init__(self, robot_name, topic_suffix):
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.robot_sound = False
        topic = f"/{robot_name}/{topic_suffix}"
        self.subscriber = rospy.Subscriber(topic, RobotPub, self._callback)

    def _callback(self, msg):
        self.pos_x = msg.pos_x
        self.pos_y = msg.pos_y
        self.robot_sound = msg.robot_sound


class EmotionSubscriber:
    """Subscribe to emotion parameters."""
    
    def __init__(self):
        self.avoidant = 0.0
        self.ambivalent = 0.0
        self.subscriber = rospy.Subscriber("/emotion", Emotion, self._callback)

    def _callback(self, msg):
        self.avoidant = msg.avoidant
        self.ambivalent = msg.ambivalent
