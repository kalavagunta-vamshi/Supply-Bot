#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

def manipulator_control(q1, q2, q3, q4, q5, q6):
    rospy.init_node('manipulator_control') 
    motor_1 = rospy.Publisher('/supplybot/Robot_Shoulder_Pan_Joint/command', Float64, queue_size=10)
    motor_2 = rospy.Publisher('/supplybot/Robot_Shoulder_Lift_Joint/command', Float64, queue_size=10)
    motor_3 = rospy.Publisher('/supplybot/Robot_Elbow_Joint/command', Float64, queue_size=10)
    motor_4 = rospy.Publisher('/supplybot/Robot_Wrist_Joint_1/command', Float64, queue_size=10)
    motor_5 = rospy.Publisher('/supplybot/Robot_Wrist_Joint_2/command', Float64, queue_size=10)
    motor_6 = rospy.Publisher('/supplybot/Robot_Wrist_Joint_3/command', Float64, queue_size=10)
    

    rate = rospy.Rate(4) 

    rospy.loginfo("Data is being sent")  
    rospy.loginfo(q1) 
    rospy.loginfo(q2) 
    rospy.loginfo(q3) 
    rospy.loginfo(q4) 
    rospy.loginfo(q5) 
    rospy.loginfo(q6) 
    rospy.loginfo("-----------------------") 

    # while not rospy.is_shutdown():
    twist = Float64()

    #Initial pose
    twist.data = q1
    motor_1.publish(twist)
    twist.data = q2
    motor_2.publish(twist)
    twist.data = q3
    motor_3.publish(twist)
    twist.data = q4
    motor_4.publish(twist)
    twist.data = q5
    motor_5.publish(twist)
    twist.data = q6
    motor_6.publish(twist)
    rate.sleep()


a, d, alpha, theta, theta1, theta2, theta3, theta4, theta5, theta6, t = sp.symbols('a d alpha theta theta1 theta2 theta3 theta4 theta5 theta6 t')    

# Transformation matrix for consecutive frames
T = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
              [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
              [0, sp.sin(alpha), sp.cos(alpha), d],
              [0, 0, 0, 1]])

T01 = T.subs([(d, 0.089159), (a, 0), (alpha, sp.pi/2), (theta, theta1)])
T12 = T.subs([(d, 0), (a, -0.425), (alpha, 0), (theta, theta2)])
T23 = T.subs([(d, 0), (a, -0.39225), (alpha, 0), (theta, theta3)]) 
T34 = T.subs([(d, 0.10915), (a, 0), (alpha, sp.pi/2), (theta, theta4)])
T45 = T.subs([(d, 0.09465), (a, 0), (alpha, -sp.pi/2), (theta, theta5)])
T56 = T.subs([(d, 0.0823), (a, 0), (alpha, 0), (theta, theta6)])

# T01 = T.subs([(d, 0.089159), (a, 0), (alpha, 0), (theta, theta1)])
# T12 = T.subs([(d, 0), (a, 0.425), (alpha, sp.pi/2), (theta, theta2)])
# T23 = T.subs([(d, 0), (a, 0.39225), (alpha, 0), (theta, theta3)]) 
# T34 = T.subs([(d, 0.10915), (a, 0), (alpha, 0), (theta, theta4)])
# T45 = T.subs([(d, 0.09465), (a, 0), (alpha, sp.pi/2), (theta, theta5)])
# T56 = T.subs([(d, 0.0823), (a, 0), (alpha, -sp.pi/2), (theta, theta6)])


T02 = T01*T12
T03 = T01*T12*T23
T04 = T01*T12*T23*T34
T05 = T01*T12*T23*T34*T45
T06 = T01*T12*T23*T34*T45*T56


J0 = sp.Matrix([[sp.diff(T06[0,3],theta1),sp.diff(T06[0,3],theta2),sp.diff(T06[0,3],theta3),sp.diff(T06[0,3],theta4),sp.diff(T06[0,3],theta5),sp.diff(T06[0,3],theta6)],
               [sp.diff(T06[1,3],theta1),sp.diff(T06[1,3],theta2),sp.diff(T06[1,3],theta3),sp.diff(T06[1,3],theta4),sp.diff(T06[1,3],theta5),sp.diff(T06[1,3],theta6)],
               [sp.diff(T06[2,3],theta1),sp.diff(T06[2,3],theta2),sp.diff(T06[2,3],theta3),sp.diff(T06[2,3],theta4),sp.diff(T06[2,3],theta5),sp.diff(T06[2,3],theta6)],
               [T01[0,2],T02[0,2],T04[0,2],T05[0,2],T06[0,2],T06[0,2]],
               [T01[1,2],T02[1,2],T04[1,2],T05[1,2],T06[1,2],T06[1,2]],
               [T01[2,2],T02[2,2],T04[2,2],T05[2,2],T06[2,2],T06[2,2]]
])

omega=(2*3.14)/5
dt=0.125
time = np.arange(0, 5, dt)

x_dot=sp.Matrix([[0],
                [omega*0.1*sp.cos(omega*t)],
                [-omega*0.1*sp.sin(omega*t)],
                [0],
                [0],
                [0]])

# q=sp.Matrix([[sp.pi/2],       #initializing with joint angles
#             [-sp.pi/2],
#             [sp.pi/2],
#             [-sp.pi],
#             [-sp.pi/2],        
#             [0]])  

q=sp.Matrix([[-0.215031984094669 + 0],       #initializing with joint angles
            [0.215031984094669 - 0],
            [-8.06081303635207e-19 + sp.pi/2],
            [3.3227320700992 - sp.pi],
            [-6.08412812358377 - sp.pi/2],        
            [2.76139605348457]])  

# manipulator_control(q[0], q[1], q[2], q[3], q[4], q[5])

for i in range(0,40):
    
    if(i!=0):
        manipulator_control(q[0], q[1], q[2], q[3], q[4], q[5])

    T06_x_y_z=T06.subs([(theta1, q[0]), (theta2, q[1]), (theta3, q[2]), (theta4, q[3]), (theta5, q[4]), (theta6, q[5])])        #substituting the joint angles in final transformation matrix
    
    J0_inv=J0.subs([(theta1, q[0]), (theta2, q[1]), (theta3, q[2]), (theta4, q[3]), (theta5, q[4]), (theta6, q[5])])
    J0_inv=(J0_inv.evalf()).inv()                                                                #taking jacobian inverse with current joint angles
    
    X_ = x_dot.subs([(t,time[i])]).evalf()
    q_dot=(J0_inv * X_).evalf()
    
    q = q + q_dot *dt

if __name__ == '__main__':
    try:
        0
    except rospy.ROSInterruptException: 
        pass