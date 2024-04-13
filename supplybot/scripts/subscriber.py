#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64


def callback (data):
    rospy.loginfo(rospy.get_caller_id() + "Response %s", data.data)

def main():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber("/supplybot/F_Axle_Joint_Controller_R/command", Float64, callback)
    rospy.Subscriber("/supplybot/F_Axle_Joint_Controller_L/command", Float64, callback)
    rospy.Subscriber("/supplybot/R_Wheel_Joint_Controller_R/command", Float64, callback)
    rospy.Subscriber("/supplybot/R_Wheel_Joint_Controller_L/command", Float64, callback)
    rospy.spin()
    
            
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
