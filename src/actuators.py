#!/usr/bin/env python3
"""
MiRo actuator control for cosmetic actions, sounds, and visual displays.
"""
import os
import rospy
from std_msgs.msg import UInt32MultiArray, UInt16MultiArray, Float32MultiArray, Int16MultiArray


class TailController:
    """Control tail wagging."""
    
    def __init__(self):
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/control/cosmetic_joints"
        self.publisher = rospy.Publisher(topic, Float32MultiArray, queue_size=0)
        self.msg = Float32MultiArray()
        self.msg.data = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]

    def wag(self, value):
        """Set tail wag position (-1 to 1)."""
        self.msg.data[1] = value
        self.publisher.publish(self.msg)


class AudioController:
    """
        TODO: Change this to alejandro's AudioController (When miro makes sound)
    """
    """Control audio output."""
    
    def __init__(self):
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/control/tone"
        self.publisher = rospy.Publisher(topic, UInt16MultiArray, queue_size=0)

    def play_tone(self, freq=2000, volume=255, duration=25):
        """Play a tone with specified frequency, volume, and duration."""
        msg = UInt16MultiArray(data=[freq, volume, duration])
        self.publisher.publish(msg)


class IlluminationController:
    """Control LED illumination."""
    
    def __init__(self):
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/control/illum"
        self.publisher = rospy.Publisher(topic, UInt32MultiArray, queue_size=0)
        self.msg = UInt32MultiArray()
        self.msg.data = [0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 
                         0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]

    def set_color(self, red=0, green=0, blue=255):
        """Set all LEDs to RGB color (0-255 per channel)."""
        r, g, b = int(red), int(green), int(blue)
        color = 0xFF000000 | (r << 16) | (g << 8) | b
        self.msg.data = [color] * 6
        self.publisher.publish(self.msg)


class AudioDetector:
    """
        TODO: Change this to alejandro's AudioDetector (When miro receives sound)
    """
    
    def __init__(self):
        self.freq = 0
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/sensors/mics"
        self.subscriber = rospy.Subscriber(topic, Int16MultiArray, self._callback)

    def _callback(self, msg):
        # Simple frequency detection based on zero crossings
        data = msg.data
        if len(data) > 0:
            self.freq = abs(sum(data)) / len(data)
