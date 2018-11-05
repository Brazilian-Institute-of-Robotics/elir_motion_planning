#!/usr/bin/env python
from math import acos,cos,sin,atan2,pow,pi,asin,fabs

class elir_inverse_kinematics():
    """ This class is used to do the inverse Kinematics of Elir Robot"""
    def __init__(self):
        #Arm 1 and 2 length
        self.link1_length= 0.39
        self.link2_length = 0.233

        #Joint_1 distance from the base_link origin 
        self.x_offset = 0.14595
        self.z_offset = 0.1967

        #Joint offset according to the URDF file
        self.joint1_offset = 0.6
        self.joint2_offset = 0.6
    def theta2_calculus(self,x_des,z_des):
        """ Calculates the value of the second angle in the planar arm kinematic chain.
        :param float x_des: The desired X coordinate
        :param float z_des: The desired Z coordinate
        :return: 
        --``theta2`` Angle of the second joint of the kinematic chain."""
        x_des = fabs(x_des)    
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

    def theta1_calculus(self,x_des,z_des,theta2,elbow_up):
        """ Calculates the value of the first angle in the planar arm kinematic chain.
        :param float x_des: The desired X coordinate
        :param float z_des: The desired Z coordinate
        :param float theta2: Value of the second angle in the kinematic chain
        :param bool elbow_up: Param wich determinates arm configuration.
        :return: 
        --``theta1`` Angle of the first joint of the kinematic chain."""
        x_des = fabs(x_des)    
        #Considers the joint1_f as the origin for theta calculation
        x_des = x_des - self.x_offset
        z_des = z_des - self.z_offset
        if elbow_up == False:
            theta2 =(-theta2)
        k1 = self.link1_length + self.link2_length*cos(theta2)
        k2 = self.link2_length*sin(theta2)

        theta1 = atan2(z_des,x_des) - atan2(k2,k1)
        return theta1

    def set_link1_length(self,length):
        """ Sets the length of kinematic chain first link.
        :param float length: First link length"""    
        self.link1_length= length

    def set_link2_length(self,length):
        """ Sets the length of kinematic chain second link link.
        :param float length: Second link length""" 
        self.link2_length= length

    def set_x_offset(self,offset):
        """ Sets the X coordinate offset of the kinematic chain, since it doesn't has the same reference as Rviz.
        :param float offset: X axis offset""" 
        self.self.x_offset = offset

    def set_z_offset(self,offset):
        """ Sets the Z coordinate offset of the kinematic chain, since it doesn't has the same reference as Rviz.
        :param float offset: Z axis offset""" 
        self.self.x_offset = offset

    def inverse_kinematics(self,x_des,z_des,elbow_up):
        """ Calculates the inverse kinematics for the desired angle.
        :param float x_des: The desired X coordinate
        :param float z_des: The desired Z coordinate
        :param bool elbow_up: Arm configuration parameter
        :return: 
        --``angle_vector`` Vector containing the first and second angle of the kinematic chain, respectively."""  
        theta2 = self.theta2_calculus(x_des,z_des)
        if theta2 == False:
            return False
        theta1 = self.theta1_calculus(x_des,z_des,theta2,elbow_up)
        
        if elbow_up == True:
            theta1 = theta1 - self.joint1_offset
            theta2 = theta2 - self.joint2_offset

            theta1 = -theta1 + self.joint1_offset
            theta2 = -theta2 - self.joint2_offset
        else:
            theta1 = theta1 - self.joint1_offset
            theta2 = theta2 - self.joint2_offset

            theta1 = -theta1 + self.joint1_offset
            theta2 = theta2 - self.joint2_offset

        angle_vector = [theta1,theta2]
        return angle_vector

if __name__ == '__main__':
  x= elir_inverse_kinematics()
  x_des = 0.402984
  z_des = 0.439057
  angles = x.inverse_kinematics(x_des,z_des,False)
  print "First angle" , str(angles[0])
  print "Second angle" , str(angles[1])


  #para z positivo e configuracao elbow down
  #theta1 negativo theta 2 positivo 
  #sai theta 1 positivo e theta 2 positivo

  #para z positivo e elbow up
  #theta 1 positivo theta 2 negativo
  #sai theta 1 negativo e theta 2 positivo

  #para z negativo e elbow down
  #theta 1 positivo e theta 2 positivo
  #sai theta 1 negativo e theta 2 positivo

  #para z negativo e elbow up
  #theta 1 positivo e theta 2 negativo
  #sai theta 1 negativo e theta 2 positivo

