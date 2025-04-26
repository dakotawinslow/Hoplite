#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseStamped
import tf.transformations
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Clock
import math
from matplotlib import colors

TIMESTEP = 0.01

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

        # calculate error between current position and target position
        error_x = self.model.target_x - self.model.position_x
        error_y = self.model.target_y - self.model.position_y
        error_theta = self.model.find_angle_error()

        # Calculate slowdown factor based on distance to target
        x_rampdown_dist = 0.5 * self.model.velocity_x ** 2 / self.model.linear_acceleration
        y_rampdown_dist = 0.5 * self.model.velocity_y ** 2 / self.model.linear_acceleration
        theta_rampdown_dist = 0.5 * self.model.angular_velocity ** 2 / self.model.angular_acceleration

        # close enough
        if abs(error_x) < 1:
            self.model.velocity_x = 0.0
        # start ramping down velocity if we are close to the target
        elif abs(error_x) < x_rampdown_dist:
            self.model.velocity_x -= math.copysign(self.model.linear_acceleration * TIMESTEP, self.model.velocity_x)
        # ramp up velocity if we are far from the target
        elif abs(self.model.velocity_x) < self.model.max_velocity_x:
            self.model.velocity_x += math.copysign(self.model.linear_acceleration * TIMESTEP, error_x)
        # capped at max velocity
        else:
            self.model.velocity_x = math.copysign(self.model.max_velocity_x, error_x)

        #same for y
        if abs(error_y) < 1:
            self.model.velocity_y = 0.0
        elif abs(error_y) < y_rampdown_dist:
            self.model.velocity_y -= math.copysign(self.model.linear_acceleration * TIMESTEP, self.model.velocity_y)
        elif abs(self.model.velocity_y) < self.model.max_velocity_y:
            self.model.velocity_y += math.copysign(self.model.linear_acceleration * TIMESTEP, error_y)
        else:
            self.model.velocity_y = math.copysign(self.model.max_velocity_y, error_y)
        # same for theta
        if abs(error_theta) < 0.01:
            self.model.angular_velocity = 0.0
        elif abs(error_theta) < theta_rampdown_dist:    
            self.model.angular_velocity -= math.copysign(self.model.angular_acceleration * TIMESTEP, self.model.angular_velocity)
        elif abs(self.model.angular_velocity) < self.model.max_angular_velocity:
            self.model.angular_velocity += math.copysign(self.model.angular_acceleration * TIMESTEP, error_theta)
        else:
            self.model.angular_velocity = math.copysign(self.model.max_angular_velocity, error_theta)

        # Update the position model based on the velocity
        self.model.position_x += self.model.velocity_x * TIMESTEP
        self.model.position_y += self.model.velocity_y * TIMESTEP
        self.model.theta += self.model.angular_velocity * TIMESTEP
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



class HopliteKinematicModel:
    def __init__(self):
        "All units are mm (or mm/s or mm/s^2) and radians (or rad/s or rad/s^2)."

        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.angular_velocity = 0.0

        self.max_velocity_x = 200
        self.max_velocity_y = 200
        self.max_angular_velocity = math.pi / 2
        self.linear_acceleration = 400
        self.angular_acceleration = math.pi / 4

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
        return error

if __name__ == "__main__":
    try:
        # Create the node
        hoplite_soldier_node = HopliteSoldierNode()

        # Spin the node
        while not rospy.is_shutdown():
            hoplite_soldier_node.send_velocity_command()
            rospy.sleep(TIMESTEP)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Exception in Hoplite Soldier Node: %s", e)
        raise e
    