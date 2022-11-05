#!/usr/bin/env python
# license removed for brevity
import rospy
from test_msgs.msg import RTK
 
def talker():
     
     rospy.init_node('talker', anonymous=True)
     pub = rospy.Publisher('chatter', RTK, queue_size=10)
     rate = rospy.Rate(10) # 10hz
     msg = RTK()
     msg.longitude = 123.2121
     msg.latitude = 65.2121
     msg.velocity = 36
     while not rospy.is_shutdown():
         #hello_str = "hello world %s" % rospy.get_time()
         #rospy.loginfo(hello_str)
         msg.longitude += 0.1
         msg.latitude += 0.1
         msg.velocity += 0.1
         pub.publish(msg)
         rate.sleep()
 
if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass

