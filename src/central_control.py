#!/usr/bin/env python3
"""
Central controller - attachment model dynamics and action selection.
"""
import rospy
import numpy as np
from Attachment-Model.msg import Action
from subscribers import RobotStateSubscriber, EmotionSubscriber


class AttachmentController:
    """Central controller implementing attachment dynamics."""
    
    def __init__(self):
        rospy.init_node('central_control')
        
        # Publishers
        self.publisher = rospy.Publisher('/central_controller', Action, queue_size=0)
        
        # Subscribers
        self.child_state = RobotStateSubscriber('miro', 'child_publisher')
        self.parent_state = RobotStateSubscriber('david', 'parent_publisher')
        self.emotion = EmotionSubscriber()
        
        # Attachment model parameters
        self.a = 1.0  # Approach rate
        self.b = 0.7  # Explore rate
        self.c = 1.0  # Need accumulation rate
        self.h = 0.01  # Time step
        
        # State variables
        self.time = 0
        self.action_msg = Action()

    def _calculate_physical_distance(self, x1, y1, x2, y2):
        """Euclidean distance between two robots."""
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def _calculate_emotional_distance(self, child_sound, parent_sound):
        """Calculate emotional distance based on sound interaction."""
        if child_sound and parent_sound:
            return max(0, self.action_msg.emotional_distance - self.h)
        return min(1, self.action_msg.emotional_distance + self.h)

    def _select_action(self, need):
        """Action selection: 0=approach, 1=explore."""
        return 0 if need > 0 else 1

    def run(self):
        """Main control loop implementing attachment dynamics."""
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            self.time += self.h
            
            # Calculate physical distance (normalized to [0,1])
            phys_dist = self._calculate_physical_distance(
                self.child_state.pos_x, self.child_state.pos_y,
                self.parent_state.pos_x, self.parent_state.pos_y
            ) / 10.0
            phys_dist = min(phys_dist, 1.0)
            
            # Calculate emotional distance
            emot_dist = self._calculate_emotional_distance(
                self.child_state.robot_sound,
                self.parent_state.robot_sound
            )
            
            # Get attachment parameters
            epsilon_av = self.emotion.avoidant
            epsilon_am = self.emotion.ambivalent
            
            # Update needs using differential equations
            # Child need dynamics
            dx_child = -self.a * (phys_dist - epsilon_av)
            dy_child = self.c * (emot_dist - epsilon_am)
            child_need = self.action_msg.child_need + (dx_child + dy_child) * self.h
            
            # Parent need dynamics
            dx_parent = -self.a * (phys_dist - epsilon_av)
            dy_parent = self.c * (emot_dist - epsilon_am)
            parent_need = self.action_msg.parent_need + (dx_parent + dy_parent) * self.h
            
            # Select actions based on needs
            self.action_msg.child = self._select_action(child_need)
            self.action_msg.parent = self._select_action(parent_need)
            self.action_msg.physical_distance = phys_dist
            self.action_msg.emotional_distance = emot_dist
            self.action_msg.child_need = child_need
            self.action_msg.parent_need = parent_need
            
            self.publisher.publish(self.action_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = AttachmentController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
