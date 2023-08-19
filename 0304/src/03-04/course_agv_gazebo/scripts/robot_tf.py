#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
import tf

br = tf.TransformBroadcaster()

def link_state_callback(msg):
    global br
    names = msg.name
    poses = msg.pose
    try:
        index = names.index('course_agv::robot_base')
        pose = poses[index]
    except ValueError:
        rospy.loginfo("'robot_base' not found in linkstates'")
        return    
    pos = pose.position
    ori = pose.orientation
    br.sendTransform((pos.x,pos.y,pos.z),
                     (ori.x,ori.y,ori.z,ori.w),
                     rospy.Time.now(),
                     "robot_base",
                     "world_base")
    rospy.loginfo("IN TF: pos=[%0.2f,%0.2f,%0.2f],ori=[%0.2f,%0.2f,%0.2f,%0.2f]",pos.x,pos.y,pos.z,ori.x,ori.y,ori.z,ori.w)


if __name__ == '__main__':
    rospy.init_node('robot_tf',anonymous=True)
    rospy.Subscriber("/gazebo/link_states",LinkStates,link_state_callback)
    rospy.spin()
