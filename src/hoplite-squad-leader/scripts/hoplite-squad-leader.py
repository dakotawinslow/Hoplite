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

class HopliteSquadLeaderNode:
    """ Squad leader is responsible for interpreting the input commands and deciding where the soldiers
    go. It receives position updates from each soldier (or the mocap system) and sends pose commands to them 
    based on the input control point and current operating mode."""
    def __init__(self):
        # Initialize the node
        rospy.init_node("hoplite_squad_leader", anonymous=False)
        self.name = rospy.get_name().split("/")[-1]

        # Don't forget that soldier_count is a GLOBAL param
        self.soldier_count = rospy.get_param("soldier_count", default=1)

        # make soldier models
        self.soldier_models = []
        self.soldier_subscribers = []
        self.soldier_publishers = []
        for i in range(self.soldier_count):
            soldier_name = "hoplite" + str(i)
            self.soldier_models.append(SoldierModel())
            self.soldier_subscribers.append(rospy.Subscriber('/' + soldier_name + '/_position', 
                                                             Marker, self.soldier_callback, callback_args=i))
            self.soldier_publishers.append(rospy.Publisher('/' + soldier_name + '/_pose', 
                                                           PoseStamped, queue_size=10))
            
        # Subscriber to the control pose
        self.pose_subscriber = rospy.Subscriber('/joystick_controller/marker', Marker, self.issue_orders)
        # Publish the current goal
        self.goal_publisher = rospy.Publisher('/squad_goal', Marker, queue_size=10)
        
        self.mode = 0 # 0: regular polygons, 1: Circle the wagons
        self.start_angle = 0.


    def soldier_callback(self, msg, soldier_index):
        """ Callback function for the soldier's position update. It receives the position update from the soldier
        and sends the pose command to the soldier. """
        self.soldier_models[soldier_index].pose = msg.pose

    def issue_orders(self, msg):
        """ Callback function for the control pose. It receives the control pose from the user and sends the pose
        command to the soldiers. """

        # Publish the current goal
        goal_marker = Marker()
        goal_marker.header.frame_id = "base_link"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.type = Marker.ARROW
        goal_marker.action = Marker.ADD
        goal_marker.pose = msg.pose
        goal_marker.scale.x = 150
        goal_marker.scale.y = 15
        goal_marker.scale.z = 15
        marker_color = "grey"
        marker_color = colors.to_rgba(marker_color)
        goal_marker.color.r = marker_color[0]
        goal_marker.color.g = marker_color[1]
        goal_marker.color.b = marker_color[2]
        goal_marker.color.a = marker_color[3]
        self.goal_publisher.publish(goal_marker)


        if self.soldier_count == 1:
            # If there is only one soldier, send the pose command directly to the soldier
            self.soldier_models[0].pose = msg.pose
            self.soldier_publishers[0].publish(msg)
            return
        target_pose = msg.pose
        if self.mode == 0:
            self.regular_polygon_formation(target_pose)
        elif self.mode == 1:
            self.circled_wagons_formation(target_pose)

        for i in range(self.soldier_count):
            self.soldier_publishers[i].publish(PoseStamped(pose=self.soldier_models[i].pose))

    def regular_polygon_formation(self, target_pose):
        """ This function is responsible for the regular polygon formation. It receives the control pose from the user
        and sends the pose command to the soldiers. """

        spacing = 1000 # mm between soldiers
        radius = spacing / (2 * np.cos((self.soldier_count - 2) * np.pi / (2 * self.soldier_count)))
        angle = 2 * np.pi / self.soldier_count
        target_angle = tf.transformations.euler_from_quaternion([target_pose.orientation.x,
                                                                 target_pose.orientation.y,
                                                                 target_pose.orientation.z,
                                                                 target_pose.orientation.w])[2]
        self.start_angle = target_angle + np.pi / 2

        for i in range(self.soldier_count):
            soldier_pose = Pose()
            soldier_pose.position.x = target_pose.position.x + radius * np.cos(((i * angle) + self.start_angle))
            soldier_pose.position.y = target_pose.position.y + radius * np.sin((i * angle) + self.start_angle)
            soldier_pose.position.z = 0.0
            # soldier_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, i * angle))
            soldier_pose.orientation = target_pose.orientation
            self.soldier_models[i].pose = soldier_pose
            # self.soldier_publishers[i].publish(PoseStamped(pose=soldier_pose))

    def circled_wagons_formation(self, target_pose):
        spacing = 1000 # mm between soldiers
        radius = spacing / (2 * np.cos((self.soldier_count - 2) * np.pi / (2 * self.soldier_count)))
        angle = 2 * np.pi / self.soldier_count

        RPM = 2
        rate = 10
        sleeper = rospy.Rate(rate)
        tick_angle = 2 * math.pi / (rate * 60 / RPM)
        while self.mode == 1:
            for i in range(self.soldier_count):
                soldier_pose = Pose()
                soldier_pose.position.x = target_pose.position.x + radius * np.cos(((i * angle) + self.start_angle))
                soldier_pose.position.y = target_pose.position.y + radius * np.sin((i * angle) + self.start_angle)
                soldier_pose.position.z = 0.0
                soldier_pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, i * angle))
                # soldier_pose.orientation = target_pose.orientation
                self.soldier_models[i].pose = soldier_pose
                self.soldier_publishers[i].publish(PoseStamped(pose=self.soldier_models[i].pose))
            self.start_angle += tick_angle
            self.start_angle = self.start_angle % (2 * math.pi)
            sleeper.sleep()



    

class SoldierModel:
    """ This class is responsible for the soldier's model. It receives the pose command from the squad leader
    and sends the position update to the squad leader. It also has a controller that takes care of the soldier's
    movement."""
    def __init__(self):
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0
        self.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
        self.minimum_spacing = 200 # mm

if __name__ == "__main__":
    try:
        node = HopliteSquadLeaderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass