import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *

rospy.wait_for_service("/robot/SendTraj")
sendTraj = rospy.ServiceProxy("/robot/SendTraj", SendState)



