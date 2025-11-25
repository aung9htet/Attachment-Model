#!/usr/bin/env python3
"""
Child robot node - reactive attachment behavior.
"""
import os
import rospy
import numpy as np
from datetime import datetime
from Attachment-Model.msg import RobotPub
from subscribers import OdomSubscriber, ActionSubscriber
from behaviors import Explorer, AprilTagTracker
from actuators import TailController, IlluminationController, AudioDetector


class ChildRobot:
    """Child robot with proximity-seeking attachment behavior."""
    
    def __init__(self):
        rospy.init_node('child_node')
        
        # Publishers
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/child_publisher"
        self.publisher = rospy.Publisher(topic, RobotPub, queue_size=0)
        
        # State subscribers and controllers
        self.odom = OdomSubscriber()
        self.action_sub = ActionSubscriber()
        self.explorer = Explorer()
        self.tag_tracker = AprilTagTracker(target_tag_id=0)
        self.tail = TailController()
        self.lights = IlluminationController()
        self.audio_detector = AudioDetector()
        
        # State variables
        self.state_msg = RobotPub()
        self.robot_found = False
        self.start_time = rospy.get_rostime()
        self.odom_history = np.empty((0, 3), float)
        
        rospy.on_shutdown(self._save_data)

    def _detect_sound(self):
        """Simple sound detection based on frequency."""
        return self.audio_detector.freq > 2000

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
            action = self.action_sub.child
            
            # Update state message
            self.state_msg.pos_x = self.odom.posx
            self.state_msg.pos_y = self.odom.posy
            self.state_msg.robot_sound = self._detect_sound()
            
            # Update LED color
            self._update_color()
            
            if action == 0:  # Approach mode
                if not self.robot_found:
                    self.robot_found = self.tag_tracker.search()
                
                if self.state_msg.robot_sound:
                    self._wag_tail()
            else:  # Explore mode
                self.explorer.min_wall_distance = 0.11
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
        filename = f"../odom_data/child_odom_{timestamp}.npy"
        np.save(filename, self.odom_history)


if __name__ == '__main__':
    try:
        robot = ChildRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
