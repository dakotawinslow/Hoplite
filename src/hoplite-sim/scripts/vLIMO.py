#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
import tf.transformations
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Clock
import math
from matplotlib import colors


PI = math.pi

class vLIMO_Node:
    def __init__(self):
        # Initialize the node
        # self.name = "vLIMO_1" #TODO Make this a parameter
        rospy.init_node("unspecified_vLIMO", anonymous=False)
        self.name = rospy.get_name().split("/")[-1]

        self.controller_node = rospy.get_param("~controller_node", default="unspecified_hoplite_controller")
        
        # Subscriber to the Twist message (cmd_vel or your topic) #TODO Make this a parameter
        self.twist_subscriber = rospy.Subscriber('/' + self.controller_node + '/cmd_vel', Twist, self.twist_callback)
        self.clock_subscriber = rospy.Subscriber('/clock', Clock, self.clock_tick)
        
        # Publisher to the vector (x, y, theta) message
        self.vector_publisher = rospy.Publisher('/' + self.controller_node +'/_position', Marker, queue_size=10)

        self.x = rospy.get_param("~x", default=0.0)
        self.y = rospy.get_param("~y", default=0.0)
        self.theta = rospy.get_param("~theta", default=0.0)
        self.vx = 0.0 # velocity in units/s
        self.vy = 0.0
        self.vtheta = 0.0 # velocity in rad/s
        self.vx_target = 0.0
        self.vy_target = 0.0
        self.vtheta_target = 0.0
        self.vx_max = 1000
        self.vy_max = 1000
        self.vtheta_max = PI/2
        self.linear_acceleration = 1000 # units/s^2
        self.angular_acceleration = 10 # rad/s^2
        self.clock_time = 0.
        self.prior_step_clock_time = 0.
        self.clock_time = rospy.Time.now()
        self.prior_step_clock_time = rospy.Time.now()

        self.color = rospy.get_param("~color", default="red")
        self.color = colors.to_rgba(self.color)

    def clock_tick(self, msg):
        self.clock_time = rospy.Time.now()   
        elapsed_time = (self.clock_time - self.prior_step_clock_time).to_sec()    

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
        self.x += self.vx * elapsed_time
        self.y += self.vy * elapsed_time
        self.theta += self.vtheta * elapsed_time
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
        marker.color.r = self.color[0]
        marker.color.g = self.color[1]
        marker.color.b = self.color[2]
        marker.color.a = self.color[3]

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

        self.prior_step_clock_time = self.clock_time

    def twist_callback(self, msg):
        self.vx_target = msg.linear.x
        self.vy_target = msg.linear.y
        self.vtheta_target = msg.angular.z

if __name__ == '__main__':
    try:
        node = vLIMO_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass