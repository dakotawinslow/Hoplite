#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
hoplite_soldier.py

A ROS1 Melodic node for running a hoplite‐style robot:
  - maintains a kinematic model (mm + rad units)
  - smoothly drives toward a target pose
  - publishes cmd_vel + viz markers
"""

from __future__ import print_function

import math
import numpy as np
import re  # Added for regex matching

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from matplotlib import colors


# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------
DEFAULT_FREQUENCY = 50  # Hz
FRAME_ID = "base_link"


# -----------------------------------------------------------------------------
# Potential Field Collision Avoidance
# -----------------------------------------------------------------------------
class PotentialFieldCollisionAvoidance(object):
    """
    Implementation of Potential Field method for collision avoidance.
    Creates repulsive forces around other robots to prevent collisions.
    """
    def __init__(self, robot_radius, detection_radius, max_velocity=None):
        """
        Initialize the Potential Field collision avoidance system.
        
        Args:
            robot_radius: Radius of the robot (mm)
            detection_radius: Maximum distance to consider for repulsive forces (mm)
            max_velocity: Maximum allowed velocity magnitude (mm/s)
        """
        self.robot_radius = robot_radius
        self.detection_radius = detection_radius
        self.max_velocity = max_velocity
        
        # Potential field parameters (can be tuned)
        self.repulsive_coeff = 500000.0  # Strength of repulsive forces
        self.distance_power = 1.        # How quickly force decreases with distance (usually 1 or 2)
        
        # Data structures for visualization
        self.viz_data = {
            'original_velocity': None,
            'adjusted_velocity': None,
            'repulsive_forces': [],
            'neighbor_info': []
        }
    
    def compute_velocity_adjustment(self, pos, vel, neighbors):
        """
        Compute velocity adjustment based on potential field method.
        
        Args:
            pos: Robot position as (x, y)
            vel: Current robot velocity as (vx, vy)
            neighbors: List of tuples, each containing:
                       (neighbor_pos, neighbor_vel, neighbor_radius)
                       
        Returns:
            Adjusted velocity as (vx, vy) to avoid collisions
        """
        if not neighbors:
            return vel  # No neighbors, no adjustment needed
        # if we are stopped, don't wiggle
        if math.hypot(*vel) < 1:
            return vel
            
        adjusted_vel = list(vel)  # Create a mutable copy of velocity
        
        # Reset visualization data
        self.viz_data = {
            'original_velocity': vel,
            'adjusted_velocity': None,
            'repulsive_forces': [],
            'neighbor_info': []
        }
        
        # Calculate repulsive forces from all neighbors
        total_force_x = 0.0
        total_force_y = 0.0
        
        for neighbor_pos, neighbor_vel, neighbor_radius in neighbors:
            # Vector from our position to neighbor position
            rel_pos_x = neighbor_pos[0] - pos[0]
            rel_pos_y = neighbor_pos[1] - pos[1]
            dist = math.hypot(rel_pos_x, rel_pos_y)

            # calculate repulsion from radius line instead of center
            dist = dist - self.robot_radius
            
            # Store neighbor info for visualization
            self.viz_data['neighbor_info'].append({
                'position': neighbor_pos,
                'velocity': neighbor_vel,
                'radius': neighbor_radius,
                'distance': dist
            })
            
            # Skip if the neighbor is too far away
            if dist > self.detection_radius:
                continue
                
            # Calculate repulsive force (inversely proportional to distance)
            # Avoid division by zero
            if dist < 1:
                dist = 1
                
            # Calculate repulsive force magnitude (increases as distance decreases)
            # Force increases as we get closer to the safety distance
            force_magnitude = self.repulsive_coeff / (dist ** self.distance_power)
            
            # Normalize direction vector (away from obstacle)
            force_dir_x = -rel_pos_x / dist
            force_dir_y = -rel_pos_y / dist
                
            # Calculate force components
            force_x = force_magnitude * force_dir_x
            force_y = force_magnitude * force_dir_y
            
            # Add to total force
            total_force_x += force_x
            total_force_y += force_y
            
            # Store for visualization
            self.viz_data['repulsive_forces'].append({
                'position': pos,
                'force': (force_x, force_y),
                'distance': dist
            })
        
        # Apply the total force to adjust velocity
        adjusted_vel[0] += total_force_x
        adjusted_vel[1] += total_force_y 
        
        # Check if velocity exceeds maximum allowed velocity
        if self.max_velocity is not None:
            velocity_magnitude = math.hypot(adjusted_vel[0], adjusted_vel[1])
            if velocity_magnitude > self.max_velocity:
                # Scale down to maximum velocity while preserving direction
                scale_factor = self.max_velocity / velocity_magnitude
                adjusted_vel[0] *= scale_factor
                adjusted_vel[1] *= scale_factor

        # Scale the adjusted velocity to the same magnitude as the desired velocity
        desired_velocity_magnitude = math.hypot(vel[0], vel[1])
        if desired_velocity_magnitude > 0:
            current_velocity_magnitude = math.hypot(adjusted_vel[0], adjusted_vel[1])
            if current_velocity_magnitude > 0:
                scale_factor = desired_velocity_magnitude / current_velocity_magnitude
                adjusted_vel[0] *= scale_factor
                adjusted_vel[1] *= scale_factor
        
        # Store the final adjusted velocity for visualization
        self.viz_data['adjusted_velocity'] = tuple(adjusted_vel)
        
        return tuple(adjusted_vel)
    
    def get_visualization_data(self):
        """Return the collected visualization data."""
        return self.viz_data


# -----------------------------------------------------------------------------
# Kinematic model (pure math, no ROS)
# -----------------------------------------------------------------------------
class HopliteKinematicModel(object):
    """
    Simple 2D kinematic model in mm and radians.
    All state updates and velocity/acceleration computations live here.
    """

    def __init__(self):
        # pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # last pose
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        # velocities
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # limits & accelerations
        self.max_v = 500.0
        self.max_omega = math.pi 
        self.linear_acc = 1000.0
        self.angular_acc = 2 * math.pi 

        # target
        self.tx = 0.0
        self.ty = 0.0
        self.ttheta = 0.0
        
        # Initialize PotentialField
        self.potential_field = None

        # PD controller parameters
        self.linear_kp = 1.0  # Proportional gain for linear velocity
        self.linear_kd = 0.1  # Derivative gain for linear velocity
        self.angular_kp = 1.0  # Proportional gain for angular velocity
        self.angular_kd = 0.1  # Derivative gain for angular velocity

        # Deadzones
        self.dist_deadzone = 15.0  # mm
        self.angle_deadzone = 0.1  # radians

        
    def set_collision_avoidance(self, robot_radius, neighbor_dist, time_horizon):
        """Set up the collision avoidance system with the given parameters."""
        # Use PotentialField for collision avoidance
        self.potential_field = PotentialFieldCollisionAvoidance(
            robot_radius,
            neighbor_dist,  # Using neighbor_dist as detection radius
            self.max_v
        )

    def set_target(self, pose):
        """Set a new target pose (Pose message)."""
        self.tx = pose.position.x
        self.ty = pose.position.y
        _, _, self.ttheta = tf.transformations.euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        )

    def angle_error(self):
        """Compute signed smallest‐angle difference to target heading."""
        err = self.ttheta - self.theta
        if err > math.pi:
            err -= 2 * math.pi
        elif err < -math.pi:
            err += 2 * math.pi
        return err

    def update(self, dt, neighbors=None):
        """
        Advance the model by dt seconds:
         - compute new vx,vy,omega
         - integrate to update x,y,theta
        
        Args:
            dt: Time step in seconds
            neighbors: List of neighbors data for collision avoidance, each as
                      (neighbor_position, neighbor_velocity, neighbor_radius)
        
        Returns a Twist message ready to publish.
        """
        # --- linear velocity control ---
        # vector to target
        ex, ey = self.tx - self.x, self.ty - self.y
        last_ex, last_ey = self.last_x - self.x, self.last_y - self.y
        delta_ex, delta_ey = ex - last_ex, ey - last_ey
        dist = math.hypot(ex, ey)
        delta_error_mag = math.hypot(delta_ex, delta_ey)

        angle_to_goal = math.atan2(ey, ex)

        v_norm = math.hypot(self.vx, self.vy)
        v_angle = math.atan2(self.vy, self.vx) if v_norm > 0 else angle_to_goal

        #  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # Depricated ramping control system, replaced with PD controller

        # ramp‐down distance
        # ramp_down = 0.5 * (v_norm ** 2) / self.linear_acc

        # # decide new speed magnitude
        # if dist < 10.0:
        #     new_speed = 0.0
        # elif dist < ramp_down:
        #     new_speed = v_norm - self.linear_acc * dt
        # elif v_norm < self.max_v:
        #     new_speed = v_norm + self.linear_acc * dt
        # else:
        #     new_speed = self.max_v

        # new_dir = angle_to_goal

        # set vx, vy
        # self.vx = new_speed * math.cos(new_dir)
        # self.vy = new_speed * math.sin(new_dir)
        
        # if self.potential_field and neighbors:
        #     pos = (self.x, self.y)
        #     vel = (self.vx, self.vy)
        #     adjusted_vel = self.potential_field.compute_velocity_adjustment(pos, vel, neighbors)
        #     self.vx, self.vy = adjusted_vel

        # # --- angular velocity control ---
        # th_err = self.angle_error()
        # w = abs(self.omega)
        # ang_ramp = 0.5 * (w ** 2) / self.angular_acc

        # if abs(th_err) < 0.01:
        #     w_new = 0.0
        # elif abs(th_err) < ang_ramp:
        #     w_new = w - self.angular_acc * dt
        # elif w < self.max_omega:
        #     w_new = w + self.angular_acc * dt
        # else:
        #     w_new = self.max_omega

        # self.omega = w_new * np.sign(th_err)

        # --- integrate pose ---
        # self.x += self.vx * dt
        # self.y += self.vy * dt
        # self.theta = (self.theta + self.omega * dt) % (2 * math.pi)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # PD controller for linear velocity
        vel_mag = self.kp * dist + self.kd * delta_error_mag

        # Limit the velocity magnitude to max_v
        if vel_mag > self.max_v:
            vel_mag = self.max_v

        # Deadzone
        if dist < self.dist_deadzone:
            vel_mag = 0.0

        # do collision avoidance if enabled
        if self.potential_field and neighbors:
            pos = (self.x, self.y)
            vel = (vel_mag * math.cos(angle_to_goal), vel_mag * math.sin(angle_to_goal))
            adjusted_vel = self.potential_field.compute_velocity_adjustment(pos, vel, neighbors)
            self.vx, self.vy = adjusted_vel
        else:
            self.vx = vel_mag * math.cos(angle_to_goal)
            self.vy = vel_mag * math.sin(angle_to_goal)

        # PD controller for angular velocity
        th_err = self.angle_error()
        ang_vel = self.angular_kp * th_err + self.angular_kd * (th_err - (self.last_theta - self.theta)) / dt
        # Limit the angular velocity to max_omega
        if abs(ang_vel) > self.max_omega:
            ang_vel = self.max_omega * np.sign(ang_vel)
        # Deadzone 
        if abs(th_err) < self.angle_deadzone:
            ang_vel = 0.0
        self.omega = ang_vel

        # correct for robot facing
        self.local_vx = self.vx * math.cos(self.theta) + self.vy * math.sin(self.theta)
        self.local_vy = -self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta) 

        # build Twist
        twist = Twist()
        twist.linear.x = self.local_vx
        twist.linear.y = self.local_vy
        twist.angular.z = self.omega
        return twist

    def current_pose(self):
        """Return a geometry_msgs/Pose of the current state."""
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        p.orientation = Quaternion(*quat)
        return p

    def target_pose(self):
        """Return a geometry_msgs/Pose of the target state."""
        quat = tf.transformations.quaternion_from_euler(0, 0, self.ttheta)
        p = Pose()
        p.position.x = self.tx
        p.position.y = self.ty
        p.orientation = Quaternion(*quat)
        return p


# -----------------------------------------------------------------------------
# Squad Member Tracking
# -----------------------------------------------------------------------------
class SquadMember(object):
    """
    Maintains information about another soldier in the squad.
    Tracks position and velocity data.
    """
    
    def __init__(self, soldier_id):
        self.soldier_id = soldier_id
        self.pose = None  # Will store the latest Pose
        self.twist = None  # Will store the latest Twist
        self.last_pose_time = None
        self.last_vel_time = None
        self.last_position_time = None  # For tracking when we last received a Marker update
        
    def update_pose(self, pose_msg, timestamp):
        """Update the soldier's position from a PoseStamped message."""
        self.pose = pose_msg.pose
        self.last_pose_time = timestamp
        
    def update_velocity(self, twist_msg, timestamp):
        """Update the soldier's velocity from a Twist message."""
        self.twist = twist_msg
        self.last_vel_time = timestamp
        
    def update_from_marker(self, marker_msg, timestamp):
        """Update the soldier's position from a Marker message."""
        if self.pose is None:
            self.pose = Pose()
            
        # Update position and orientation from marker
        self.pose.position = marker_msg.pose.position
        self.pose.orientation = marker_msg.pose.orientation
        self.last_position_time = timestamp
        
    def get_position(self):
        """Return position as (x, y, theta) if available, None otherwise."""
        if self.pose is None:
            return None
        
        x = self.pose.position.x
        y = self.pose.position.y
        _, _, theta = tf.transformations.euler_from_quaternion(
            [self.pose.orientation.x, self.pose.orientation.y,
             self.pose.orientation.z, self.pose.orientation.w]
        )
        return (x, y, theta)
        
    def get_velocity(self):
        """Return velocity as (vx, vy, omega) if available, None otherwise."""
        if self.twist is None:
            return None
            
        vx = self.twist.linear.x
        vy = self.twist.linear.y
        omega = self.twist.angular.z
        return (vx, vy, omega)

# -----------------------------------------------------------------------------
# Helper for making RViz markers
# -----------------------------------------------------------------------------
class MarkerFactory(object):
    @staticmethod
    def make_arrow(pose, rgba, scale, frame_id=FRAME_ID):
        """
        Create an ARROW marker at `pose` with RGBA tuple and scale tuple (x,y,z).
        """
        m = Marker()
        m.header.frame_id = frame_id
        m.type = Marker.ARROW
        m.pose = pose
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.scale.x, m.scale.y, m.scale.z = scale
        return m

    @staticmethod
    def make_cylinder(pose, rgba, radius, height, id=0, frame_id=FRAME_ID):
        """
        Create a CYLINDER marker at `pose` with RGBA tuple, radius and height.
        """
        m = Marker()
        m.header.frame_id = frame_id
        m.id = id
        m.type = Marker.CYLINDER
        m.pose = pose
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.scale.x = radius * 2  # diameter in x
        m.scale.y = radius * 2  # diameter in y
        m.scale.z = height      # height in z
        m.action = Marker.ADD
        m.lifetime = rospy.Duration()  # Persistent
        return m

    @staticmethod
    def make_vector_marker(start_point, vector, rgba=(1.0, 0.0, 0.0, 1.0), 
                        scale=(20.0, 5.0, 5.0), id=0, ns="vectors", frame_id=FRAME_ID):
        """
        Create an arrow marker representing a vector starting from start_point.
        
        Args:
            start_point: (x, y) tuple for the start position
            vector: (vx, vy) tuple for the vector direction and magnitude
            rgba: Color tuple (r, g, b, a)
            scale: Scale tuple (shaft_length, shaft_diameter, head_diameter)
            id: Marker ID
            ns: Namespace for the marker
            frame_id: Frame ID
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Starting point
        marker.pose.position.x = start_point[0]
        marker.pose.position.y = start_point[1]
        marker.pose.position.z = 0
        
        # Calculate orientation from vector
        angle = math.atan2(vector[1], vector[0])
        quat = tf.transformations.quaternion_from_euler(0, 0, angle)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Scale the arrow length by vector magnitude
        magnitude = math.hypot(vector[0], vector[1])
        marker.scale.x = magnitude  # Length
        marker.scale.y = scale[1]   # Width
        marker.scale.z = scale[2]   # Height
        
        # Set the color
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        marker.lifetime = rospy.Duration(0.5)  # Short-lived
        
        return marker
    
    @staticmethod
    def make_sphere_marker(position, radius, rgba=(1.0, 0.0, 0.0, 0.5), id=0, 
                        ns="spheres", frame_id=FRAME_ID):
        """
        Create a sphere marker at the given position.
        
        Args:
            position: (x, y) tuple for the position
            radius: Radius of the sphere
            rgba: Color tuple (r, g, b, a)
            id: Marker ID
            ns: Namespace for the marker
            frame_id: Frame ID
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2
        
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    @staticmethod
    def make_line_strip_marker(points, rgba=(1.0, 0.0, 0.0, 1.0), width=2.0, id=0,
                            ns="lines", frame_id=FRAME_ID):
        """
        Create a line strip marker connecting the given points.
        
        Args:
            points: List of (x, y) tuples
            rgba: Color tuple (r, g, b, a)
            width: Width of the line
            id: Marker ID
            ns: Namespace for the marker
            frame_id: Frame ID
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = width  # Line width
        
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        # Add points
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0
            marker.points.append(p)
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    @staticmethod
    def make_text_marker(position, text, rgba=(1.0, 1.0, 1.0, 1.0), height=100.0, id=0,
                      ns="text", frame_id=FRAME_ID):
        """
        Create a text marker at the given position.
        
        Args:
            position: (x, y) tuple for the position
            text: Text to display
            rgba: Color tuple (r, g, b, a)
            height: Height of the text
            id: Marker ID
            ns: Namespace for the marker
            frame_id: Frame ID
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        
        marker.scale.z = height  # Text height
        
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        
        marker.text = text
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker

    @staticmethod
    def rgba_from_name(name):
        """Use matplotlib to convert a CSS color name to RGBA (0–1 floats)."""
        return colors.to_rgba(name)


# -----------------------------------------------------------------------------
# ROS node (wires model + factory into pubs/subs)
# -----------------------------------------------------------------------------
class HopliteSoldierNode(object):
    def __init__(self):
        # init node
        rospy.init_node("hoplite_soldier", anonymous=False)
        self.name = rospy.get_name().lstrip("/")

        # parameters
        freq = rospy.get_param("~frequency", DEFAULT_FREQUENCY)

        # Get soldier count from parameter server
        self.soldier_count = rospy.get_param("/soldier_count", 0)
        self.squad_members = {}
        
        # Get our own ID (from the node name)
        match = re.search(r'hoplite(\d+)', self.name)
        if match:
            self.soldier_id = int(match.group(1))
        else:
            self.soldier_id = None
            rospy.logwarn("Could not determine own soldier ID from name: %s", self.name)
        
        # Collision avoidance parameters
        self.robot_radius = rospy.get_param("~robot_radius", 200.0)  # mm
        self.neighbor_dist = rospy.get_param("~neighbor_dist", 1500.0)  # mm
        self.time_horizon = rospy.get_param("~time_horizon", 3.0)  # seconds
        self.use_collision_avoidance = rospy.get_param("~use_collision_avoidance", True)  # Enable/disable
        self.repulsive_coeff = rospy.get_param("~repulsive_coeff", 5000000.0)  # Force strength
        self.distance_power = rospy.get_param("~distance_power", 2.0)  # Distance decay factor
        
        rospy.loginfo("Collision avoidance: enabled=%s, radius=%.1f, neighbor_dist=%.1f, time_horizon=%.1f",
                     self.use_collision_avoidance, self.robot_radius, self.neighbor_dist, self.time_horizon)
        
        # publishers & subscribers
        self.cmd_pub = rospy.Publisher("/{}/cmd_vel".format(self.name),
                                       Twist, queue_size=10)
        self.pose_pub = rospy.Publisher("/{}/position_estimate".format(self.name),
                                        Marker, queue_size=10)
        self.tgt_pub = rospy.Publisher("/{}/target".format(self.name),
                                       Marker, queue_size=10)
        # Add cylinder visualization publishers
        self.robot_radius_pub = rospy.Publisher("/{}/robot_radius".format(self.name),
                                             Marker, queue_size=10)
        self.neighbor_dist_pub = rospy.Publisher("/{}/neighbor_dist".format(self.name),
                                             Marker, queue_size=10)
        # Add collision avoidance visualization publisher
        self.avoidance_viz_pub = rospy.Publisher("/{}/collision_avoidance".format(self.name),
                                               MarkerArray, queue_size=10)

        # Flag to track if we've received our initial position
        self.initial_position_received = False
        
        # model + timing
        self.model = HopliteKinematicModel()
        # Configure collision avoidance
        if self.use_collision_avoidance:
            self.model.set_collision_avoidance(self.robot_radius, self.neighbor_dist, self.time_horizon)
            if hasattr(self.model.potential_field, 'repulsive_coeff'):
                self.model.potential_field.repulsive_coeff = self.repulsive_coeff
            if hasattr(self.model.potential_field, 'distance_power'):
                self.model.potential_field.distance_power = self.distance_power
        
        self.last_time = rospy.Time.now()
        self.last_squad_log = rospy.Time.now()

        # Add subscriber for ground truth position - do this early to get initial position
        position_topic = "/{}/_position".format(self.name)
        rospy.Subscriber(position_topic, Marker, self.on_position_update)
        rospy.loginfo("Waiting for initial position from %s", position_topic)
        
        # Wait for initial position before continuing
        timeout = rospy.Duration(10.0)  # 10 second timeout
        start_wait = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz for checking
        
        while not self.initial_position_received and not rospy.is_shutdown():
            if (rospy.Time.now() - start_wait) > timeout:
                rospy.logwarn("Timeout waiting for initial position. Starting without it.")
                break
            rate.sleep()
            
        # Continue with initialization after receiving position or timeout
        # subscribers for our own information
        rospy.Subscriber("/{}/_pose".format(self.name),
                         PoseStamped, self.on_target)
        # rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_target)
        
        # Setup squad tracking
        self.setup_squad_tracking()
        self.setup_own_position_tracking()

        # timer for control loop - start after initialization
        rospy.Timer(rospy.Duration(1.0 / freq), self.on_timer)
        rospy.loginfo("HopliteSoldierNode started at %d Hz", freq)

    def setup_squad_tracking(self):
        """Setup subscribers for tracking other squad members."""
        if self.soldier_count <= 1:
            rospy.loginfo("No other squad members to track (soldier_count = %d)", self.soldier_count)
            return
            
        for i in range(self.soldier_count):
            # Skip ourselves
            if i == self.soldier_id:
                continue
                
            # Create squad member object
            squad_member = SquadMember(i)
            self.squad_members[i] = squad_member
            
            # Subscribe to pose, position, and velocity
            position_topic = "/hoplite{0}/_position".format(i)
            vel_topic = "/hoplite{0}/cmd_vel".format(i)
            
            rospy.Subscriber(position_topic, Marker, 
                            lambda msg, sid=i: self.on_squad_position(msg, sid))
            rospy.Subscriber(vel_topic, Twist, 
                            lambda msg, sid=i: self.on_squad_velocity(msg, sid))
            
            rospy.loginfo("Tracking squad member %d (pose, position, velocity)", i)
            
    def setup_own_position_tracking(self):
        """Setup subscriber for tracking own actual position."""
        # We now subscribe to _position in the constructor, so this method can be simplified
        # but we keep the _pose subscription for backward compatibility
        topic = "/{}/_pose".format(self.name)
        rospy.Subscriber(topic, PoseStamped, self.on_target)
        rospy.loginfo("Monitoring own pose from %s", topic)

    def on_own_target(self, msg):
        """Callback when our own actual position is updated."""

        # for both target setting and position updating
        self.model.set_target(msg.pose)
        
        # publish a magenta arrow at the target
        rgba = MarkerFactory.rgba_from_name("magenta")
        marker = MarkerFactory.make_arrow(
            self.model.target_pose(), rgba, scale=(200, 50, 50))
        self.tgt_pub.publish(marker)
    

    def on_position_update(self, marker_msg):
        """Callback when our own actual position is updated via Marker."""
        # Move the current position to last position
        self.model.last_x = self.model.x
        self.model.last_y = self.model.y
        # Extract position and orientation from the marker message
        x = marker_msg.pose.position.x
        y = marker_msg.pose.position.y
        _, _, theta = tf.transformations.euler_from_quaternion(
            [marker_msg.pose.orientation.x, marker_msg.pose.orientation.y,
             marker_msg.pose.orientation.z, marker_msg.pose.orientation.w]
        )
        
        # Update the internal model with the actual position
        self.model.x = x
        self.model.y = y
        self.model.theta = theta
        
        # If this is our first position update, also set it as our target
        if not self.initial_position_received:
            self.model.tx = x
            self.model.ty = y
            self.model.ttheta = theta
            self.initial_position_received = True
            rospy.loginfo("Initial position received: x=%.1f y=%.1f θ=%.2f", x, y, theta)
            
            # Publish a target marker at our initial position
            rgba = MarkerFactory.rgba_from_name("magenta")
            marker = MarkerFactory.make_arrow(
                self.model.target_pose(), rgba, scale=(200, 50, 50))
            self.tgt_pub.publish(marker)
        
        # Log the update at debug level
        rospy.logdebug("Updated own position from marker: x=%.1f y=%.1f θ=%.2f", x, y, theta)
            
    def on_squad_position(self, msg, soldier_id):
        """Callback for when another soldier's position marker is updated."""
        if soldier_id in self.squad_members:
            self.squad_members[soldier_id].update_from_marker(msg, rospy.Time.now())
            rospy.logdebug("Updated position for soldier %d from marker", soldier_id)

    def on_squad_velocity(self, msg, soldier_id):
        """Callback for when another soldier's velocity is updated."""
        if soldier_id in self.squad_members:
            self.squad_members[soldier_id].update_velocity(msg, rospy.Time.now())
            
    def log_squad_status(self):
        """Log the current status of all squad members."""
        if not self.squad_members:
            return
        
        rospy.loginfo("Squad status:")
        for soldier_id, member in self.squad_members.items():
            pos = member.get_position()
            vel = member.get_velocity()
            
            if pos is not None:
                x, y, theta = pos
                rospy.loginfo("  Soldier %d: pos=(%.1f, %.1f, %.2f)", 
                             soldier_id, x, y, theta)
            
            if vel is not None:
                vx, vy, omega = vel
                rospy.loginfo("  Soldier %d: vel=(%.1f, %.1f, %.2f)", 
                             soldier_id, vx, vy, omega)

    def on_target(self, msg):
        """Callback when a new PoseStamped target arrives."""
        self.model.set_target(msg.pose)
        # publish a magenta arrow at the target
        rgba = MarkerFactory.rgba_from_name("magenta")
        marker = MarkerFactory.make_arrow(
            self.model.target_pose(), rgba, scale=(200, 50, 50))
        self.tgt_pub.publish(marker)
        rospy.loginfo("New target: x=%.1f y=%.1f θ=%.2f",
                      self.model.tx, self.model.ty, self.model.ttheta)

    def get_neighbor_data(self):
        """
        Get data about neighbors for collision avoidance.
        
        Returns:
            List of tuples, each containing (position, velocity, radius) for a neighbor
        """
        neighbors = []
        for member_id, member in self.squad_members.items():
            pos = member.get_position()
            vel = member.get_velocity()
            
            if pos is not None and vel is not None:
                position = (pos[0], pos[1])  # x, y
                velocity = (vel[0], vel[1])  # vx, vy
                neighbors.append((position, velocity, self.robot_radius))
                
        return neighbors

    def on_timer(self, event):
        """Control loop: compute dt, update model, publish cmd_vel + markers."""
        now = event.current_real
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # Get neighbor data for collision avoidance
        neighbors = self.get_neighbor_data() if self.use_collision_avoidance else []
        
        # compute and send velocity (with collision avoidance)
        twist = self.model.update(dt, neighbors)
        self.cmd_pub.publish(twist)

        # publish current pose marker (green)
        green = (0.0, 1.0, 0.0, 1.0)
        pm = MarkerFactory.make_arrow(
            self.model.current_pose(), green, scale=(250, 20, 20))
        self.pose_pub.publish(pm)
        
        # Publish robot radius as semi-transparent yellow cylinder
        yellow = (1.0, 1.0, 0.0, 0.3)  # Semi-transparent yellow
        robot_radius_marker = MarkerFactory.make_cylinder(
            self.model.current_pose(), yellow, self.robot_radius, 50, id=1)
        self.robot_radius_pub.publish(robot_radius_marker)
        
        # Publish neighbor detection distance as very transparent blue cylinder
        blue = (0.0, 0.0, 1.0, 0.15)  # Very transparent blue
        neighbor_dist_marker = MarkerFactory.make_cylinder(
            self.model.current_pose(), blue, self.neighbor_dist, 30, id=2)
        self.neighbor_dist_pub.publish(neighbor_dist_marker)
        
        # Visualize collision avoidance if enabled
        if self.use_collision_avoidance and self.model.potential_field:
            self.publish_avoidance_visualization()
        
        # Log squad status approximately every 5 seconds
        if (now - self.last_squad_log).to_sec() >= 5.0:
            self.log_squad_status()
            self.last_squad_log = now

    def publish_avoidance_visualization(self):
        """Publish visualization of collision avoidance forces and data."""
        if not hasattr(self.model.potential_field, 'viz_data'):
            return
        
        viz_data = self.model.potential_field.viz_data
        marker_array = MarkerArray()
        marker_id = 0
        
        # Current position
        pos = (self.model.x, self.model.y)
        
        # Original velocity vector (green)
        if viz_data['original_velocity']:
            orig_vel = viz_data['original_velocity']
            marker_id += 1
            original_vel_marker = MarkerFactory.make_vector_marker(
                pos, orig_vel,
                rgba=(0.0, 1.0, 0.0, 1.0),  # Green
                scale=(1.0, 5.0, 5.0),
                id=marker_id,
                ns="original_velocity"
            )
            marker_array.markers.append(original_vel_marker)
            
            # Add text label
            marker_id += 1
            text_pos = (pos[0] + orig_vel[0]/2, pos[1] + orig_vel[1]/2)
            magnitude = math.hypot(orig_vel[0], orig_vel[1])
            text_marker = MarkerFactory.make_text_marker(
                text_pos,
                "Original: {:.1f} mm/s".format(magnitude),
                rgba=(1.0, 1.0, 1.0, 1.0),
                height=50.0,
                id=marker_id,
                ns="original_velocity_text"
            )
            marker_array.markers.append(text_marker)
        
        # Adjusted velocity vector (blue)
        if viz_data['adjusted_velocity']:
            adj_vel = viz_data['adjusted_velocity']
            marker_id += 1
            adjusted_vel_marker = MarkerFactory.make_vector_marker(
                pos, adj_vel,
                rgba=(0.0, 0.0, 1.0, 1.0),  # Blue
                scale=(1.0, 5.0, 5.0),
                id=marker_id,
                ns="adjusted_velocity"
            )
            marker_array.markers.append(adjusted_vel_marker)
            
            # Add text label
            marker_id += 1
            text_pos = (pos[0] + adj_vel[0]/2, pos[1] + adj_vel[1]/2)
            magnitude = math.hypot(adj_vel[0], adj_vel[1])
            text_marker = MarkerFactory.make_text_marker(
                text_pos,
                "Adjusted: {:.1f} mm/s".format(magnitude),
                rgba=(1.0, 1.0, 1.0, 1.0),
                height=50.0,
                id=marker_id,
                ns="adjusted_velocity_text"
            )
            marker_array.markers.append(text_marker)
        
        # Repulsive forces (red)
        for force_data in viz_data['repulsive_forces']:
            marker_id += 1
            position = force_data['position']
            force = force_data['force']
            
            # Scale force for visualization (forces can be very large)
            force_scale = 0.001  # Adjust this value to make arrows visible but not too large
            scaled_force = (force[0] * force_scale, force[1] * force_scale)
            
            force_marker = MarkerFactory.make_vector_marker(
                position, scaled_force,
                rgba=(1.0, 0.0, 0.0, 0.8),  # Red
                scale=(1.0, 3.0, 3.0),
                id=marker_id,
                ns="repulsive_force"
            )
            marker_array.markers.append(force_marker)
        
        # Neighbor info (cylinders for each neighbor)
        for i, info in enumerate(viz_data['neighbor_info']):
            marker_id += 1
            neighbor_pos = info['position']
            neighbor_radius = info['radius']
            distance = info['distance']
            
            # Neighbor position as cylinder
            neighbor_pose = Pose()
            neighbor_pose.position.x = neighbor_pos[0]
            neighbor_pose.position.y = neighbor_pos[1]
            neighbor_pose.orientation.w = 1.0
            
            # Color based on distance (red if close, yellow if medium, green if far)
            # Normalize distance within detection radius
            norm_dist = min(1.0, distance / self.model.potential_field.detection_radius)
            
            if norm_dist < 0.33:
                color = (1.0, 0.0, 0.0, 0.5)  # Red (close)
            elif norm_dist < 0.66:
                color = (1.0, 1.0, 0.0, 0.5)  # Yellow (medium)
            else:
                color = (0.0, 1.0, 0.0, 0.5)  # Green (far)
                
            neighbor_marker = MarkerFactory.make_cylinder(
                neighbor_pose, color, neighbor_radius, 30, id=marker_id
            )
            marker_array.markers.append(neighbor_marker)
            
            # Add text showing distance
            marker_id += 1
            text_pos = (neighbor_pos[0], neighbor_pos[1] + neighbor_radius + 50)
            text_marker = MarkerFactory.make_text_marker(
                text_pos,
                "Dist: {:.1f}mm".format(distance),
                rgba=(1.0, 1.0, 1.0, 1.0),
                height=40.0,
                id=marker_id
            )
            marker_array.markers.append(text_marker)
        
        # Publish all visualization markers
        if marker_array.markers:
            self.avoidance_viz_pub.publish(marker_array)


if __name__ == "__main__":
    try:
        HopliteSoldierNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception: %s", e)
        raise
