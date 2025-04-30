# MoCAP & OptiTrack Integration

This folder documents our process for integrating OptiTrack-based motion capture with the LIMObot swarm system.

## Overview

- OptiTrack cameras track each LIMObot’s 3D position in real time.
- Motive software on the MoCAP computer streams this data to ROS2 topics.
- A ROS1 bridge converts these ROS2 topics to ROS1 for use by the LIMObots.
- The master LIMObot uses this localization data to coordinate formation movement.

## Files

- `LIMOBOT_Swarm_Motive.jpg`: Screenshot of 3 LIMObots tracked in Motive.
- `MoCAP_Position_streaming.jpg`: Screenshot showing streamed ROS2 topics from MoCAP.
- `README.md`: This documentation file.

## Notes

- Make sure the MoCAP machine is connected to the same ROS network.
- Run the `ros2` side of the bridge on the MoCAP computer for best performance.
