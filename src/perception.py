#!/usr/bin/env python3
"""
Perception modules for AprilTag and audio detection.
"""
import rospy
import cv2
import numpy as np


class AprilTagPerception:
    """Simplified AprilTag detection."""
    
    def __init__(self):
        try:
            import apriltag
            self.detector = apriltag.Detector()
        except ImportError:
            rospy.logwarn("AprilTag library not available")
            self.detector = None

    def detect_tags(self, image):
        """Detect AprilTags in image and return list of detections."""
        if self.detector is None:
            return []
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)
        
        tags = []
        for detection in detections:
            tag = Tag(
                tag_id=detection.tag_id,
                center=detection.center,
                corners=detection.corners
            )
            tags.append(tag)
        return tags


class Tag:
    """Simple tag dataclass."""
    
    def __init__(self, tag_id, center, corners):
        self.tag_id = tag_id
        self.center = center
        self.corners = corners
