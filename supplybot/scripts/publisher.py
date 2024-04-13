#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def spin_truck():
    rospy.init_node('publisher', anonymous=True)
    pub_right = rospy.Publisher('/supplybot/F_Axle_Joint_Controller_R/command', Float64, queue_size=10) # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
    pub_left = rospy.Publisher('/supplybot/F_Axle_Joint_Controller_L/command', Float64, queue_size=10)
    pub_move_left= rospy.Publisher('/supplybot/R_Wheel_Joint_Controller_R/command', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'
    pub_move_right = rospy.Publisher('/supplybot/R_Wheel_Joint_Controller_L/command', Float64, queue_size=10) # Add your topic for move here '' Eg '/my_robot/longitudinal_controller/command'
    
    rate = rospy.Rate(100)
    control_speed = "10.0"
    control_turn = "10"
    

    while not rospy.is_shutdown():
        pub_right.publish(0) # publish the turn command.
        pub_left.publish(0) # publish the turn command.
        pub_move_left.publish(-10) # publish the control speed.
        pub_move_right.publish(-10)
        print(pub_move_left)
        print(pub_move_right)
        


        
if __name__ == '__main__':
    try:
        spin_truck()
    except rospy.ROSInterruptException:
        pass
