#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
import math
from std_srvs.srv import SetBool
from std_msgs.msg import Float64

class armControl:
    def __init__(self):
        rospy.init_node("arm_control_node")
    
        self.srv_nav_start = rospy.Service(
            '/mov_start', SetBool, self.mov_start)
        self.arm_shoulder_pan_pub = rospy.Publisher(
            "/ur5_arm/shoulder_pan_joint_position_controller/command",Float64,queue_size=1000)
        
        self.arm_shoulder_pan_position = Float64()
        self.is_mov_start = False

        self.arm_shoulder_pan_position.data = 3.14
    
    def mov_start(self, req):
        if not self.is_mov_start:
            if req.data:
                print("Action Stations!!!")
                self.is_mov_start = True
                return True, "Start move"
            else:
                print("Ignoring request as the \"data\" field is set to false")
                return False, "False received"
        else:
            if req.data:
                print("Starting failed: already running")
                return False, "Already running"
            else:
                return False, "doing nothing" 
        
    def flow(self):
        if self.is_mov_start == True:
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
