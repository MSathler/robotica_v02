#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import JointState
import std_msgs.msg


def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('teste', anonymous=True)
    h = std_msgs.msg.Header()
    h.seq = 1
    h.stamp = rospy.Time.now()
    h.frame_id = ""
    
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        hello_str = [0,0]
        #hello_str.velocity = [0,0]
        #hello_str.effort = [0,0]
        rospy.loginfo(hello_str)
        pub.publish(h,0,hello_str,str(0),str(0))
        #pub.publish(hello_str)      
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
