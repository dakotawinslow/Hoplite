# Hoplite

A system for coordinated movement of AgileX LIMO Robots using a gamepad-style controller and ROS-based motion capture.

## Roadmap:

- Establish basic manuverability (single robot 3-axis movement w/ bluetooth controller)
    - In Progress

- Establish position feedback with motion capture system
    - Display robot $(x,y)$ location as points moving around virtual space

- Simulate 4-bot coordination in square formation
    - Bots should move as single square formation where each bot is a corner
    - Fixed spacing
    - Virtual "Control Point" directs center of mass
    - Leader bot receives controller inputs, calculates target locations for all bots
    - Each bot moves to target location, receiving updates about its location from the simulator

    - Things I have learned
        - There is a robust simulation environment already built in gazebo!
        - ... but it only support 4-wheel-diff and Ackerman mode, making it useless here. sad.
        - I think we might be able to leverage the basic structure, but modify 4-wheel diff mode to instead move by raw vectors (which is how we command the motion platform in mecanum mode anyway)

- Port simulation to mocap space
    - Replace simulator data with Mocap data while retaining same logic and heirarchy