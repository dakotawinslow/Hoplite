import rospy
from rospy.m

class control_point(object):
  def __init__(self):
    rospy.init_node('joystick_controller', anonymous=False)
    self.x = 0
    self.y = 0
    self.theta = 0
    
    self.joy_subscriber = rospy.Subscriber('/joy', 
    
  def 
    