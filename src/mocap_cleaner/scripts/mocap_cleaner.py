#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped, Pose
import tf.transformations

class MocapCleanerNode:
    """ Node to clean up the mocap data by removing the first and last frames. """
    def __init__(self):
        # Initialize the node
        rospy.init_node("mocap_cleaner", anonymous=False)
        self.name = rospy.get_name().split("/")[-1]

        # Subscriber to the mocap data
        subscribe_topic = rospy.get_param("~subscribe_topic", "/UNSPECIFIED_subscribe_topic")
        self.mocap_subscriber = rospy.Subscriber(subscribe_topic, PoseStamped, self.mocap_callback)
        
        publish_topic = rospy.get_param("~publish_topic", "/UNSPECIFIED_publish_topic")
        # Publisher for the cleaned mocap data
        self.cleaned_mocap_publisher = rospy.Publisher(publish_topic, PoseStamped, queue_size=10)

    def mocap_callback(self, msg):
        
        # remove z coordinate
        msg.pose.position.z = 0.0

        # remove all rotation except yaw
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        euler = (0.0, 0.0, euler[2])
        quaternion = tf.transformations.quaternion_from_euler(*euler)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        # Publish the cleaned mocap data
        self.cleaned_mocap_publisher.publish(msg)
        rospy.loginfo("Published cleaned mocap data: %s", msg)

if __name__ == "__main__":
    try:
        node = MocapCleanerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

