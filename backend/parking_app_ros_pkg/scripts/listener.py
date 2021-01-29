#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    ros_root_node = 'listener'
    rospy.init_node(ros_root_node, anonymous=True)
    subscribed_topic_parking_spots = 'parkingSpots'
    rospy.Subscriber(subscribed_topic_parking_spots, String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

