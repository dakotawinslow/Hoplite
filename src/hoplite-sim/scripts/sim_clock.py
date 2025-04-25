#!/usr/bin/python2
import rospy
from std_msgs.msg import Float32MultiArray
from rosgraph_msgs.msg import Clock

import time


def clock_publisher():
    # Initialize the node
    rospy.init_node('sim_clock', anonymous=True)

    # Get the period parameter (default to 0.1 seconds)
    period = rospy.get_param('~period', 0.1)

    # Create a publisher for the /clock topic
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(1.0 / period)

    # Initialize the clock time
    clock_time = 0.0

    rospy.loginfo("Initiated clock publisher with period: " + str(period))


    while not rospy.is_shutdown():
        # Create the clock message
        clock_msg = Clock()
        clock_msg.clock = rospy.Time(clock_time)

        # Publish the clock message
        clock_pub.publish(clock_msg)

        # Log the clock time
        # rospy.loginfo("Clock time: " + str(clock_time) )

        # Increment the clock time
        clock_time += period

        # Sleep for the remaining time
        # rate.sleep()
        time.sleep(period)


if __name__ == '__main__':
    try:
        clock_publisher()

    except rospy.ROSInterruptException:
        pass