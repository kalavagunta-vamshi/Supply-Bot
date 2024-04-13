#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def manipulator_control():
    rospy.init_node('manipulator_control') 
    motor_1 = rospy.Publisher('/supplybot/Robot_Shoulder_Pan_Joint/command', Float64, queue_size=10)
    motor_2 = rospy.Publisher('/supplybot/Robot_Shoulder_Lift_Joint/command', Float64, queue_size=10)
    motor_3 = rospy.Publisher('/supplybot/Robot_Elbow_Joint/command', Float64, queue_size=10)
    motor_4 = rospy.Publisher('/supplybot/Robot_Wrist_Joint_1/command', Float64, queue_size=10)
    motor_5 = rospy.Publisher('/supplybot/Robot_Wrist_Joint_2/command', Float64, queue_size=10)
    motor_6 = rospy.Publisher('/supplybot/Robot_Wrist_Joint_3/command', Float64, queue_size=10)
    
    
    rate = rospy.Rate(1) 
    rospy.loginfo("Data is being sent")  
    while not rospy.is_shutdown():
        twist = Float64()

              
        #Initial pose
        twist.data = 0 * 0.01744
        motor_1.publish(twist)
        twist.data = -45* 0.01744
        motor_2.publish(twist)
        twist.data = 90 * 0.01744
        motor_3.publish(twist)
        twist.data = -45 * 0.01744
        motor_4.publish(twist)
        twist.data = 90 * 0.01744
        motor_5.publish(twist)
        twist.data = 0 * 0.01744
        motor_6.publish(twist)
        rate.sleep()

        
        

if __name__ == '__main__':
    try:
        manipulator_control()
    except rospy.ROSInterruptException: 
        pass
