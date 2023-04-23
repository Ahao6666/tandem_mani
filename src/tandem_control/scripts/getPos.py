#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

import os

def doMsg1(msg):
    #print(msg)
    with open("posinfo1", "a") as f:
        f.write(str(msg) + '\n')
def doMsg2(msg):
    #print(msg)
    with open("posinfo2", "a") as f:
        f.write(str(msg) + '\n')
def doMsg3(msg):
    #print(msg)
    with open("posinfo3", "a") as f:
        f.write(str(msg) + '\n')

def doMsg4(msg):
    #print(msg)
    with open("posinfo4", "a") as f:
        f.write(str(msg) + '\n')
def doMsg5(msg):
    #print(msg)
    with open("posinfo5", "a") as f:
        f.write(str(msg) + '\n')

if __name__ =="__main__":
    rospy.init_node("getpos")
    
    sub1 = rospy.Subscriber("/joint1_pos",Float64, doMsg1, queue_size=10)
    sub2 = rospy.Subscriber("/joint1_pos",Float64, doMsg2, queue_size=10)
    sub3 = rospy.Subscriber("/joint1_pos",Float64, doMsg3, queue_size=10)
    sub4 = rospy.Subscriber("/joint1_pos",Float64, doMsg4, queue_size=10)
    sub5 = rospy.Subscriber("/joint1_pos",Float64, doMsg5, queue_size=10)
    
    rospy.spin()
    