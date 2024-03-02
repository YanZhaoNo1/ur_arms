#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float64

class armControl:
    def __init__(self):
        rospy.init_node("arm_control_node")
    
        self.start_subscrber = rospy.Subscriber(
            "/mov_start", Bool, self.mov_start)
        self.arm_shoulder_pan_pub = rospy.Publisher(
            "/ur5_arm/shoulder_pan_joint_position_controller/command",Float64,queue_size=1000)
        
        self.arm_shoulder_pan_position = Float64()
        self.is_mov_start = False

        self.arm_shoulder_pan_position.data = 3.14
    
    def mov_start(self):
        if not self.is_mov_start:
            print("Action Stations!!!")
            self.is_mov_start = True
            return True
        else:
            print("Starting failed: already running")
            return False
        
    def flow(self):
        # if self.is_mov_start == True:
        self.arm_shoulder_pan_pub.publish(self.arm_shoulder_pan_position)
    
    def run(self):
        rate = rospy.Rate(20)
        self.flow()
        while not rospy.is_shutdown():
            self.flow()
            rate.sleep()    

if __name__ == "__main__":
    try:
        arm_control = armControl()
        arm_control.run()

    except rospy.ROSInterruptException:
        pass
