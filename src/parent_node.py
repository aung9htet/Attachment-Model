#!/usr/bin/env python3
"""
Parent robot node - caregiving attachment behavior.
"""
import os
import rospy
import numpy as np
from datetime import datetime
from Attachment-Model.msg import RobotPub
from subscribers import OdomSubscriber, ActionSubscriber
from behaviors import Explorer, AprilTagTracker
from actuators import TailController, IlluminationController, AudioController


class ParentRobot:
    """Parent robot with caregiving attachment behavior."""
    
    def __init__(self):
        rospy.init_node('parent_node')
        
        # Publishers
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/parent_publisher"
        self.publisher = rospy.Publisher(topic, RobotPub, queue_size=0)
        
        # State subscribers and controllers
        self.odom = OdomSubscriber()
        self.action_sub = ActionSubscriber()
        self.explorer = Explorer()
        self.tag_tracker = AprilTagTracker(target_tag_id=0)
        self.tail = TailController()
        self.lights = IlluminationController()
        self.audio = AudioController()
        
        # State variables
        self.state_msg = RobotPub()
        self.robot_found = False
        self.start_time = rospy.get_rostime()
        self.odom_history = np.empty((0, 3), float)
        
        rospy.on_shutdown(self._save_data)

    def _update_color(self):
        """Update LED color based on emotional distance."""
        emotional_dist = self.action_sub.emotional_distance
        red = emotional_dist * 255
        green = (1 - emotional_dist) * 255
        self.lights.set_color(red=red, green=green, blue=0)

    def _wag_tail(self):
        """Sinusoidal tail wagging pattern."""
        time_elapsed = rospy.get_rostime().secs - self.start_time.secs
        wag_value = np.sin(time_elapsed * 10)
        self.tail.wag(wag_value)

    def run(self):
        """Main control loop."""
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            action = self.action_sub.parent
            
            # Update state message
            self.state_msg.pos_x = self.odom.posx
            self.state_msg.pos_y = self.odom.posy
            
            # Update LED color
            self._update_color()
            
            if action == 0:  # Approach mode
                self.audio.play_tone(freq=2000, volume=255, duration=25)
                self.state_msg.robot_sound = True
                
                if not self.robot_found:
                    self.robot_found = self.tag_tracker.search()
                elif self.robot_found:
                    self._wag_tail()
            else:  # Explore mode
                self.explorer.min_wall_distance = 0.2
                self.explorer.explore()
                self.state_msg.robot_sound = False
                self.robot_found = False
            
            # Publish state and record position
            self.publisher.publish(self.state_msg)
            found_flag = 1 if self.robot_found else 0
            self.odom_history = np.vstack([
                self.odom_history,
                [self.state_msg.pos_x, self.state_msg.pos_y, found_flag]
            ])
            
            rate.sleep()

    def _save_data(self):
        """Save odometry data on shutdown."""
        timestamp = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
        filename = f"../odom_data/parent_odom_{timestamp}.npy"
        np.save(filename, self.odom_history)


if __name__ == '__main__':
    try:
        robot = ParentRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
