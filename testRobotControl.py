#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
def talker():
    pub = rospy.Publisher('aquashoko_chatter', JointState, queue_size=10)
    rospy.init_node('aquashoko_talker')
    rate = rospy.Rate(10) # 10hz
    hello_str = JointState()
    hello_str.header = Header()
    hello_str.header.stamp = rospy.Time.now()
    hello_str.name = ['joint00', 'joint01', 'joint02', 'joint10', 'joint11', 'joint12', 'joint20', 'joint21', 'joint22', 'joint30', 'joint31','joint32']
    hello_str.position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)

    i = 0
    while not rospy.is_shutdown():
      hello_str.header.stamp = rospy.Time.now()
      pub.publish(hello_str)
      rate.sleep()
      print i
      i = i+1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
