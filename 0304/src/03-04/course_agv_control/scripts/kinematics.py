#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

right_wheel_vel_pub = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command',Float64,queue_size=10)
left_wheel_vel_pub = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command',Float64,queue_size=10)
r = 0.08
l = 0.1
A = np.array([[r/2,     r/2],
             [r/(2*l), -r/(2*l)]])
invA = np.linalg.inv(A)

def TwistCallback(msg):
    global right_wheel_vel_pub 
    global left_wheel_vel_pub
    # [v] = [r/2   r/2 ]  @ [omegaright]
    # [w]   [r/2l -r/2l]    [omegaleft ]
    v = msg.linear.x
    w = msg.angular.z
    robot_speed = np.array([v,w])
    wheel_speed = np.dot(invA,robot_speed)
    rspeed = Float64()
    rspeed.data = wheel_speed[0]
    lspeed = Float64()
    lspeed.data = wheel_speed[1]
    right_wheel_vel_pub.publish(rspeed)
    left_wheel_vel_pub.publish(lspeed)
    rospy.loginfo("velocity is v = %0.2f, w = %0.2f",v,w)
    rospy.loginfo("wheelspeed is r = %0.2f, l = %0.2f",rspeed.data,lspeed.data)

def TwistSuscriber():
    rospy.Subscriber("/course_agv/velocity",Twist,TwistCallback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('kinematics',anonymous=True)
    TwistSuscriber()
