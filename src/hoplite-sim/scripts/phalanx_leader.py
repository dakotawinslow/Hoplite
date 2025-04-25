import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from custom_msgs.msg import (
    Pose2DArray,
    Pose2D,
)  # Replace with your custom message types if needed


class PackLeaderNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("pack_leader_node", anonymous=True)

        # Parameters
        self.num_nodes = rospy.get_param("~num_nodes", 5)  # Number of nodes to command

        # Subscribers
        self.input_sub = rospy.Subscriber("/input_topic", Pose2D, self.input_callback)

        # Publishers
        self.command_pubs = [
            rospy.Publisher(f"/node_{i}/command", Pose2D, queue_size=10)
            for i in range(self.num_nodes)
        ]

    def input_callback(self, msg):
        """
        Callback function for processing incoming messages.
        :param msg: Pose2D message containing x, y, and theta
        """
        rospy.loginfo(f"Received input: x={msg.x}, y={msg.y}, theta={msg.theta}")

        # Example logic: Distribute the input to `n` nodes (modify as needed)
        for i, pub in enumerate(self.command_pubs):
            command_msg = Pose2D()
            command_msg.x = msg.x + i  # Example: Offset x by node index
            command_msg.y = msg.y + i  # Example: Offset y by node index
            command_msg.theta = msg.theta  # Keep the same theta
            pub.publish(command_msg)
            rospy.loginfo(
                f"Published to node_{i}: x={command_msg.x}, y={command_msg.y}, theta={command_msg.theta}"
            )

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = PackLeaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
