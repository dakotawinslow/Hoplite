#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseStamped
import tf.transformations
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Clock
import math
from matplotlib import colors
import numpy as np

FREQUENCY = 100

class HopliteSoldierNode:
    """ The brain of the hoplite bots. Keeps an internal model of it's state and pulishes
    commands to the motion platform (or simulator node). Accepts location targeting commands 
    from the platoon leader and publishes it's current state on /{name}/location. Also can 
    read location data from the motion capture system to update it's internal model.
    """

    def __init__(self):
        # Initialize the node
        rospy.init_node("Hoplite_Soldier", anonymous=False)
        self.name = rospy.get_name().split("/")[-1]

        # Publisher to the vector (x, y, theta) message
        self.command_publisher = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=10)
        self.position_estimate_publisher = rospy.Publisher('/' + self.name + '/position_estimate', Marker, queue_size=10)
        self.target_publisher = rospy.Publisher('/' + self.name + '/target', Marker, queue_size=10)

        # self.position_subscriber = rospy.Subscriber('/' + self.name + '/position', Pose, self.set_target_position)
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.set_target_position)

        self.model = HopliteKinematicModel()
        self.clock_time = rospy.Time.now()
        self.prior_step_clock_time = rospy.Time.now()

    def set_target_position(self, msg):
        """ Set the target position for the hoplite soldier. This is the position that the
        hoplite soldier will try to reach.
        """
        # make us tolerant of both Pose and PoseStamped messages
        if type(msg) == PoseStamped:
            msg = msg.pose

        self.model.set_target_pose(msg)

        # Publish the target position
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.ARROW
        marker.pose = self.model.get_target_pose()
        color = colors.to_rgba("Magenta")
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.scale.x = 200
        marker.scale.y = 50
        marker.scale.z = 50
        self.target_publisher.publish(marker)

    def send_velocity_command(self):
        """ Send a velocity command to the hoplite soldier. This is the command that the
        hoplite soldier will use to move.
        """
        self.clock_time = rospy.Time.now()
        elapsed_time = (self.clock_time - self.prior_step_clock_time).to_sec()

        # calculate error vector
        error = np.array([self.model.target_x - self.model.position_x,
                          self.model.target_y - self.model.position_y])
        
        # Calculate the distance to the target
        distance = np.linalg.norm(error)

        # Calculate the angle to the target
        angle_to_target = np.arctan2(error[1], error[0])

        current_velocity_norm = math.sqrt(self.model.velocity_x ** 2 + self.model.velocity_y ** 2)
        current_velocity_angle = np.arctan2(self.model.velocity_y, self.model.velocity_x)
        angle_error = angle_to_target - current_velocity_angle
        # Normalize the angle error to the range [-pi, pi]
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        rampdown_dist = 0.5 * current_velocity_norm ** 2 / self.model.linear_acceleration

        # Calculate the new velocity based on the distance to the target (for smooth stops)
        if distance < 1:
            new_velocity = 0.0
        elif distance < rampdown_dist:
            new_velocity = current_velocity_norm - self.model.linear_acceleration * elapsed_time
        elif current_velocity_norm < self.model.max_velocity_x:
            new_velocity = current_velocity_norm + self.model.linear_acceleration * elapsed_time
        else:
            new_velocity = self.model.max_velocity_x

        # calculate swingover angle (ignore if we are very slow or stationary)
        if abs(current_velocity_norm) < 0.1:
            new_velocity_angle = angle_to_target
        elif abs(angle_error) < 0.01:
            new_velocity_angle = current_velocity_angle 
        else:
            new_velocity_angle = current_velocity_angle + (self.model.turnaround_acceleration * elapsed_time * np.sign(angle_error))
            
        
        self.model.velocity_x = new_velocity * np.cos(new_velocity_angle)
        self.model.velocity_y = new_velocity * np.sin(new_velocity_angle)

        theta_error = self.model.find_angle_error()
        current_angular_velocity = abs(self.model.angular_velocity)
        angular_rampdown_dist = 0.5 * current_angular_velocity ** 2 / self.model.angular_acceleration
        if abs(theta_error) < 0.01:
            new_angular_velocity = 0.0
        elif abs(theta_error) < angular_rampdown_dist:
            new_angular_velocity = current_angular_velocity - self.model.angular_acceleration * elapsed_time
        elif abs(current_angular_velocity) < self.model.max_angular_velocity:
            new_angular_velocity = current_angular_velocity + self.model.angular_acceleration * elapsed_time
        else:
            new_angular_velocity = self.model.max_angular_velocity
        self.model.angular_velocity = new_angular_velocity * np.sign(theta_error)

        # Update the position model based on the velocity
        self.model.position_x += self.model.velocity_x * elapsed_time
        self.model.position_y += self.model.velocity_y * elapsed_time
        self.model.theta += self.model.angular_velocity * elapsed_time
        self.model.theta = self.model.theta % (2 * math.pi)
        # Publish the new velocity command
        twist = Twist()
        twist.linear.x = self.model.velocity_x
        twist.linear.y = self.model.velocity_y
        twist.angular.z = self.model.angular_velocity
        self.command_publisher.publish(twist)
        # Publish our position estimate
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.ARROW
        marker.pose = self.model.get_current_pose()
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 250
        marker.scale.y = 40
        marker.scale.z = 40
        self.position_estimate_publisher.publish(marker)
        # Publish the target position
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = Marker.ARROW
        marker.pose = self.model.get_target_pose()
        color = colors.to_rgba("Magenta")
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.scale.x = 200
        marker.scale.y = 50
        marker.scale.z = 50
        self.target_publisher.publish(marker)

        self.prior_step_clock_time = self.clock_time





class HopliteKinematicModel:
    def __init__(self):
        "All units are mm (or mm/s or mm/s^2) and radians (or rad/s or rad/s^2)."

        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.angular_velocity = 0.0

        self.max_velocity_x = 300
        self.max_velocity_y = 300
        self.max_angular_velocity = math.pi / 2
        self.linear_acceleration = 200
        self.angular_acceleration = math.pi / 4
        self.turnaround_acceleration = math.pi / 2

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

    def get_position(self):
        return ((self.position_x, self.position_y, self.theta))
    
    def get_velocity(self):
        return ((self.velocity_x, self.velocity_y, self.angular_velocity))
    
    def get_acceleration(self):
        return ((self.acceleration_x, self.acceleration_y, self.angular_acceleration))
    
    def get_current_pose(self):
        orientation = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        pose = Pose()
        pose.position.x = self.position_x
        pose.position.y = self.position_y
        pose.orientation = Quaternion(*orientation)
        return pose
    
    def get_target_pose(self):
        orientation = tf.transformations.quaternion_from_euler(0, 0, self.target_theta)
        pose = Pose()
        pose.position.x = self.target_x
        pose.position.y = self.target_y
        pose.orientation = Quaternion(*orientation)
        return pose
    
    def set_target_pose(self, pose):
        """ Set the target position for the hoplite soldier. This is the position that the
        hoplite soldier will try to reach.
        """
        self.target_x = pose.position.x
        self.target_y = pose.position.y
        self.target_theta = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
    
    def find_angle_error(self):
        """ Find the angle error between the current angle and the target angle. """
        error = self.target_theta - self.theta
        if error > math.pi:
            error -= 2 * math.pi
        elif error < -math.pi:
            error += 2 * math.pi
        # log the error
        # rospy.loginfo("Angle error: %f", error)
        return error

if __name__ == "__main__":
    try:
        # Create the node
        hoplite_soldier_node = HopliteSoldierNode()

        # Spin the node
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            hoplite_soldier_node.send_velocity_command()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Exception in Hoplite Soldier Node: %s", e)
        raise e
    