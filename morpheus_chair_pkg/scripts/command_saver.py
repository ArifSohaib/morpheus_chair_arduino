#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from motor_driver_arduino import MotorDriver
from std_srvs.srv import Empty, EmptyRequest
import os
class RobotMover(object):

    def __init__(self):
        rospy.Subscriber("/morpheus_bot/cmd_vel", Twist, self.cmd_vel_callback)
        self.motor_driver = MotorDriver()
        rospy.wait_for_service('/raspicam_node/start_capture')

        start_cam = rospy.ServiceProxy('/raspicam_node/start_capture', Empty)
        request_e = EmptyRequest()
        start_cam(request_e)
        rospy.loginfo("Started Camera")


    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Save Values
        file_name = "/home/nvidia/catkin_ws/saved_data/cmd_vel_files.json"
        if os.path.isfile(file_name):
                f = open(file_name,'a+')
        else:
            f = open(file_name,'w+')
        #not using Python 3.6 format due to compatibility reasons
        f.write("{\"linear\":" + str(linear_speed) + ", \"angular\":" + str(angular_speed) + "},\n")
    
    def listener(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('morpheuschair_cmd_vel_saver', anonymous=True)
    robot_mover = RobotMover()
    robot_mover.listener()