#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
import tf.transformations
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Clock
import math


PI = math.pi

class vLIMO_Node:
    def __init__(self):
        # Initialize the node
        self.name = "vLIMO_1" #TODO Make this a parameter
        rospy.init_node(self.name, anonymous=True)
        
        # Subscriber to the Twist message (cmd_vel or your topic) #TODO Make this a parameter
        self.twist_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.twist_callback)
        self.clock_subscriber = rospy.Subscriber('/clock', Clock, self.clock_tick)
        
        # Publisher to the vector (x, y, theta) message
        self.vector_publisher = rospy.Publisher('/vLIMO1_position', Marker, queue_size=10)
        
        # Rate at which we will publish the vector
        self.rate = rospy.Rate(10)  # 10 Hz

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0 # velocity in units/s
        self.vy = 0.0
        self.vtheta = 0.0 # velocity in rad/s
        self.vx_target = 0.0
        self.vy_target = 0.0
        self.vtheta_target = 0.0
        self.vx_max = 200
        self.vy_max = 200
        self.vtheta_max = PI/2
        self.linear_acceleration = 400 # units/s^2
        self.angular_acceleration = PI/4 # rad/s^2
        self.time_step = 0.1 # seconds
        self.clock_time = 0.0

    def clock_tick(self, msg):
        # This function is called at each clock tick
        # we expect a clock tick every 0.1 seconds

        self.clock_time = msg.clock.to_sec()

        error_vx = self.vx_target - self.vx
        error_vy = self.vy_target - self.vy
        error_vtheta = self.vtheta_target - self.vtheta
        if abs(error_vx) > self.linear_acceleration:
            self.vx += (self.linear_acceleration if error_vx > 0 else -self.linear_acceleration)
        else:
            self.vx = self.vx_target
        if abs(error_vy) > self.linear_acceleration:
            self.vy += (self.linear_acceleration if error_vy > 0 else -self.linear_acceleration)
        else:
            self.vy = self.vy_target
        if abs(error_vtheta) > self.angular_acceleration:
            self.vtheta += (self.angular_acceleration if error_vtheta > 0 else -self.angular_acceleration)
        else:
            self.vtheta = self.vtheta_target

        # Update the position based on the velocity
        self.x += self.vx * 0.1
        self.y += self.vy * 0.1
        self.theta += self.vtheta * 0.1
        self.theta = self.theta % (2 * PI)
        rotation_quaternion = tf.transformations.quaternion_from_euler(0, 0, self.theta)


        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.ARROW
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.orientation = Quaternion(*rotation_quaternion)
        marker.scale.x = 200
        marker.scale.y = 50
        marker.scale.z = 50
        marker.color.r = 1.0
        marker.color.a = 1.0

        # Publish the vector message
        self.vector_publisher.publish(marker)
        # # Log the position
        # rospy.loginfo("Position: x=" + str(self.x) + ", y=" + str(self.y) + ", theta=" + str(self.theta))
        # # Log the velocity
        # rospy.loginfo("Velocity: vx=" + str(self.vx) + ", vy=" + str(self.vy) + ", vtheta=" + str(self.vtheta))
        # # Log the target velocity
        # rospy.loginfo("Target Velocity: vx=" + str(self.vx_target) + ", vy=" + str(self.vy_target) + ", vtheta=" + str(self.vtheta_target))
        # # Log the error
        # rospy.loginfo("Error: vx=" + str(error_vx) + ", vy=" + str(error_vy) + ", vtheta=" + str(error_vtheta))
        # # Log the time
        # rospy.loginfo("Time: " + str(self.clock_time))

    def twist_callback(self, msg):
        # This function is called when a new Twist message is received
        # Extract the linear and angular velocities from the Twist message
        self.vx_target = msg.linear.x
        self.vy_target = msg.linear.y
        self.vtheta_target = msg.angular.z

        # Log the received Twist message
        rospy.loginfo("Received Twist: linear=(" + str(self.vx_target) + ", " + str(self.vy_target) + "), angular=" + str(self.vtheta_target))
        

if __name__ == '__main__':
    try:
        node = vLIMO_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass