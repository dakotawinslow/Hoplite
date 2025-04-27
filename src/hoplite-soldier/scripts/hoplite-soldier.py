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
from geometry_msgs.msg import Twist, Quaternion, Pose, PoseStamped
from visualization_msgs.msg import Marker
from matplotlib import colors


# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------
DEFAULT_FREQUENCY = 100  # Hz
FRAME_ID = "base_link"


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

        # velocities
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # limits & accelerations
        self.max_v = 800.0
        self.max_omega = math.pi / 2.0
        self.linear_acc = 200.0
        self.angular_acc = math.pi / 4.0
        self.turn_acc = math.pi / 2.0

        # target
        self.tx = 0.0
        self.ty = 0.0
        self.ttheta = 0.0

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

    def update(self, dt):
        """
        Advance the model by dt seconds:
         - compute new vx,vy,omega
         - integrate to update x,y,theta
        Returns a Twist message ready to publish.
        """
        # --- linear velocity control ---
        # vector to target
        ex, ey = self.tx - self.x, self.ty - self.y
        dist = math.hypot(ex, ey)
        angle_to_goal = math.atan2(ey, ex)

        v_norm = math.hypot(self.vx, self.vy)
        v_angle = math.atan2(self.vy, self.vx) if v_norm > 0 else angle_to_goal

        # ramp‐down distance
        ramp_down = 0.5 * (v_norm ** 2) / self.linear_acc

        # decide new speed magnitude
        if dist < 1.0:
            new_speed = 0.0
        elif dist < ramp_down:
            new_speed = v_norm - self.linear_acc * dt
        elif v_norm < self.max_v:
            new_speed = v_norm + self.linear_acc * dt
        else:
            new_speed = self.max_v

        # decide new velocity direction
        ang_err = angle_to_goal - v_angle
        # normalize
        if ang_err > math.pi:
            ang_err -= 2 * math.pi
        elif ang_err < -math.pi:
            ang_err += 2 * math.pi

        if v_norm < 0.1:
            new_dir = angle_to_goal
        elif abs(ang_err) < 0.01:
            new_dir = v_angle
        else:
            new_dir = v_angle + self.turn_acc * dt * np.sign(ang_err)

        # set vx, vy
        self.vx = new_speed * math.cos(new_dir)
        self.vy = new_speed * math.sin(new_dir)

        # --- angular velocity control ---
        th_err = self.angle_error()
        w = abs(self.omega)
        ang_ramp = 0.5 * (w ** 2) / self.angular_acc

        if abs(th_err) < 0.01:
            w_new = 0.0
        elif abs(th_err) < ang_ramp:
            w_new = w - self.angular_acc * dt
        elif w < self.max_omega:
            w_new = w + self.angular_acc * dt
        else:
            w_new = self.max_omega

        self.omega = w_new * np.sign(th_err)

        # --- integrate pose ---
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.theta = (self.theta + self.omega * dt) % (2 * math.pi)

        # build Twist
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
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
        
    def update_pose(self, pose_msg, timestamp):
        """Update the soldier's position from a PoseStamped message."""
        self.pose = pose_msg.pose
        self.last_pose_time = timestamp
        
    def update_velocity(self, twist_msg, timestamp):
        """Update the soldier's velocity from a Twist message."""
        self.twist = twist_msg
        self.last_vel_time = timestamp
        
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
        match = re.search(r'soldier_(\d+)', self.name)
        if match:
            self.soldier_id = int(match.group(1))
        else:
            self.soldier_id = None
            rospy.logwarn("Could not determine own soldier ID from name: %s", self.name)
        
        # publishers & subscribers
        self.cmd_pub = rospy.Publisher("/{}/cmd_vel".format(self.name),
                                       Twist, queue_size=10)
        self.pose_pub = rospy.Publisher("/{}/position_estimate".format(self.name),
                                        Marker, queue_size=10)
        self.tgt_pub = rospy.Publisher("/{}/target".format(self.name),
                                       Marker, queue_size=10)

        rospy.Subscriber("/{}/_pose".format(self.name),
                         PoseStamped, self.on_target)
        
        # Setup squad tracking
        self.setup_squad_tracking()

        # model + timing
        self.model = HopliteKinematicModel()
        self.last_time = rospy.Time.now()
        self.last_squad_log = rospy.Time.now()

        # timer for control loop
        rospy.Timer(rospy.Duration(1.0 / freq), self.on_timer)
        rospy.loginfo("HopliteSoldierNode started at %d Hz", freq)

    def setup_squad_tracking(self):
        """Setup subscribers for tracking other squad members."""
        if self.soldier_count <= 1:
            rospy.loginfo("No other squad members to track (soldier_count = %d)", self.soldier_count)
            return
            
        for i in range(1, self.soldier_count + 1):
            # Skip ourselves
            if i == self.soldier_id:
                continue
                
            # Create squad member object
            squad_member = SquadMember(i)
            self.squad_members[i] = squad_member
            
            # Subscribe to pose and velocity
            pose_topic = "/soldier_{0}/_pose".format(i)
            vel_topic = "/soldier_{0}/cmd_vel".format(i)
            
            rospy.Subscriber(pose_topic, PoseStamped, 
                            lambda msg, sid=i: self.on_squad_pose(msg, sid))
            rospy.Subscriber(vel_topic, Twist, 
                            lambda msg, sid=i: self.on_squad_velocity(msg, sid))
            
            rospy.loginfo("Tracking squad member %d", i)

    def on_squad_pose(self, msg, soldier_id):
        """Callback for when another soldier's pose is updated."""
        if soldier_id in self.squad_members:
            self.squad_members[soldier_id].update_pose(msg, rospy.Time.now())
            
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

    def on_timer(self, event):
        """Control loop: compute dt, update model, publish cmd_vel + markers."""
        now = event.current_real
        dt = (now - self.last_time).to_sec()
        self.last_time = now

        # compute and send velocity
        twist = self.model.update(dt)
        self.cmd_pub.publish(twist)

        # publish current pose marker (green)
        green = (0.0, 1.0, 0.0, 1.0)
        pm = MarkerFactory.make_arrow(
            self.model.current_pose(), green, scale=(250, 20, 20))
        self.pose_pub.publish(pm)
        
        # Log squad status approximately every 5 seconds
        if (now - self.last_squad_log).to_sec() >= 5.0:
            self.log_squad_status()
            self.last_squad_log = now


if __name__ == "__main__":
    try:
        HopliteSoldierNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception: %s", e)
        raise
