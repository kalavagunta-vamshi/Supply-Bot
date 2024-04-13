#!/usr/bin/env python3
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
    #motor_7 = rospy.Publisher('/supplybot/R_Wheel_Joint_Controller_R', Float64, queue_size=10)
    #motor_8 = rospy.Publisher('/supplybot/R_Wheel_Joint_Controller_L', Float64, queue_size=10)
    pub_right = rospy.Publisher('/supplybot/F_Axle_Joint_Controller_R/command', Float64, queue_size=10)
    pub_left = rospy.Publisher('/supplybot/F_Axle_Joint_Controller_L/command', Float64, queue_size=10)
    pub_move_left= rospy.Publisher('/supplybot/R_Wheel_Joint_Controller_R/command', Float64, queue_size=10)
    pub_move_right = rospy.Publisher('/supplybot/R_Wheel_Joint_Controller_L/command', Float64, queue_size=10)
    
    rate = rospy.Rate(1) 
    rospy.loginfo("Data is being sent")  
    #while not rospy.is_shutdown():
    twist = Float64()

              
    #Initial pose
    twist.data = 0 * 0.01744
    motor_1.publish(twist)
    twist.data = -45 * 0.01744
    motor_2.publish(twist)
    twist.data = 90 * 0.01744
    motor_3.publish(twist)
    twist.data = -45 * 0.01744
    motor_4.publish(twist)
    twist.data = 90 * 0.01744
    motor_5.publish(twist)
    twist.data = 0 * 0.01744
    motor_6.publish(twist)
    print(twist)
    rate.sleep()
        
        

        
        #pick pose
    twist.data = -90 * 0.01744
    motor_1.publish(twist)
    twist.data = -45 * 0.01744
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
    rate.sleep()
        
        
    #     #place pose
    # twist.data = 0 * 0.01744
    # motor_1.publish(twist)
    # twist.data = -45 * 0.01744
    # motor_2.publish(twist)
    # twist.data = 90 * 0.01744
    # motor_3.publish(twist)
    # twist.data = -45 * 0.01744
    # motor_4.publish(twist)
    # twist.data = 90 * 0.01744
    # motor_5.publish(twist)
    # twist.data = 0 * 0.01744
    # motor_6.publish(twist)
    # rate.sleep()
        
        

if __name__ == '__main__':
    try:
        manipulator_control()
    except rospy.ROSInterruptException: 
        pass
