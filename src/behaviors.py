#!/usr/bin/env python3
"""
Robot behavior modules for movement, exploration, and tag detection.
"""
import os
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from subscribers import OdomSubscriber, RangeSubscriber
from perception import AprilTagPerception


class Movement:
    """Basic movement control for MiRo."""
    
    def __init__(self):
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/control/cmd_vel"
        self.publisher = rospy.Publisher(topic, TwistStamped, queue_size=0)
        self.cmd = TwistStamped()

    def move(self, linear=0.0, angular=0.0):
        """Set linear and angular velocity."""
        self.cmd.twist.linear.x = linear
        self.cmd.twist.angular.z = angular
        self.publisher.publish(self.cmd)

    def stop(self):
        """Stop all movement."""
        self.move(0.0, 0.0)


class Explorer:
    """Random exploration behavior with obstacle avoidance."""
    
    def __init__(self):
        self.movement = Movement()
        self.odom = OdomSubscriber()
        self.range = RangeSubscriber()
        
        self.speed = 0.2
        self.min_wall_distance = 0.105
        self.turn_angle = 0
        self.is_exploring = True
        self.start_time = rospy.get_rostime()
        self.angle_turn = np.random.choice([np.pi/3, -np.pi/3])

    def _should_turn(self):
        """Check if robot should turn to avoid obstacles."""
        time_elapsed = rospy.get_rostime().secs - self.start_time.secs
        
        if not self.is_exploring:
            if (time_elapsed % 5) == 0:
                self.is_exploring = True
            return True
        elif self.range.range < self.min_wall_distance:
            self.is_exploring = False
            return True
        return False

    def explore(self):
        """Execute exploration behavior."""
        time_elapsed = rospy.get_rostime().secs - self.start_time.secs
        
        if self._should_turn():
            if (time_elapsed % 5) == 0:
                self.angle_turn = np.random.choice([np.pi/3, -np.pi/3])
            self.movement.move(linear=0, angular=self.angle_turn * 2)
        else:
            if (time_elapsed % 2) == 0:
                max_angle = np.pi / 2
                self.turn_angle = np.random.uniform(-max_angle, max_angle)
            self.movement.move(linear=self.speed, angular=self.turn_angle)


class AprilTagTracker:
    """Track and approach AprilTags using camera."""
    
    def __init__(self, target_tag_id=0):
        self.target_tag = target_tag_id
        self.movement = Movement()
        self.bridge = CvBridge()
        self.perception = AprilTagPerception()
        self.found_tag = False
        
        topic = f"/{os.getenv('MIRO_ROBOT_NAME')}/sensors/caml/compressed"
        self.camera_sub = rospy.Subscriber(
            topic, CompressedImage, self._camera_callback,
            queue_size=1, tcp_nodelay=True
        )

    def _camera_callback(self, msg):
        """Process camera images to detect tags."""
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            tags = self.perception.detect_tags(image)
            
            for tag in tags:
                if tag.tag_id == self.target_tag:
                    self.found_tag = True
                    self._approach_tag(tag)
                    return
        except Exception as e:
            rospy.logwarn(f"Tag detection error: {e}")

    def _approach_tag(self, tag):
        """Move towards detected tag."""
        image_center = 320
        tag_center = tag.center[0]
        error = (tag_center - image_center) / image_center
        
        angular_speed = -error * 0.5
        linear_speed = 0.1 if abs(error) < 0.2 else 0.0
        
        self.movement.move(linear=linear_speed, angular=angular_speed)

    def search(self):
        """Return whether tag has been found."""
        return self.found_tag
