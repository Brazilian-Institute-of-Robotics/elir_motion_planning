#!/usr/bin/env python
#import rospy
#from std_msgs.msg  import Float64
#from std_srvs.srv import Empty
#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import acos,cos,sin,atan2,pow


class inverse_kinematics():

    def __init__(self):
        #Arm 1 and 2 length
        self.link1_length= 0.39
        self.link2_length = 0.22
        #Joint_1 distance from the base_link origin 
        self.x_offset = 0.14595
        self.z_offset = 0.07885

        #Theta class variable to me calculated
        self.theta1 = 0 
        self.theta2 = 0

        #self.open_f_claw_service = rospy.Service('f_claw/open', Empty, self.open_f_claw)
        #self.close_f_claw_service = rospy.Service('f_claw/close', Empty, self.close_f_claw)
        #self.f_claw_publisher = rospy.Publisher('f_claw_trajectory_controller/command', JointTrajectory, queue_size=10)  

        #rospy.spin()
        

    def theta2_calculus(self,x_des,z_des):
        #Calculates the theta2 angle for link2_f
        #Considers the joint1_f as the origin for theta calculation
        x_des = x_des - self.x_offset
        z_des = z_des - self.z_offset

        x_des_square = pow(x_des,2)
        z_des_square = pow(z_des,2)
        link1_length_square = pow(self.link1_length,2)
        link2_length_square = pow(self.link2_length,2)
        if (x_des_square+z_des_square-link1_length_square - link2_length_square)/(2*self.link1_length*self.link2_length) > 1:
            return False
        else:
            #The theta signal indicates if the arm is with the elbow up or the elbow down configuration
            theta2 = acos((x_des_square+z_des_square-link1_length_square - link2_length_square)/(2*self.link1_length*self.link2_length))
            return theta2

    def theta1_calculus(self,x_des,z_des,theta2):
        #Considers the joint1_f as the origin for theta calculation
        x_des = x_des - self.x_offset
        z_des = z_des - self.z_offset
        
        k1 = self.link1_length + self.link2_length*cos(theta2)
        k2 = self.link2_length*sin(theta2)

        theta1 = atan2(z_des,x_des) - atan2(k2,k1)
        return theta1
if __name__ == '__main__':
    x = inverse_kinematics()
    x_des = 0.688147696748
    z_des = 0.317909389308
    theta2 = x.theta2_calculus(x_des,z_des)
    theta1 = x.theta1_calculus(x_des,z_des,theta2)
    print "Theta 1:",str(theta1)
    print "Theta 2:",str(theta2)

    # try:
    #     rospy.init_node('claw_control_services', anonymous=True)
    #     x = claw_control_services()
    # except rospy.ROSInterruptException: pass