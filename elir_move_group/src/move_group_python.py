#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from elir_move_group.srv import MoveRequest
from kinematics_solver.solver import elir_inverse_kinematics

def all_close(goal, actual, tolerance):
  #this function is based on the move it move group tutorials
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupElir(object):
  """This class is used to create the ElirMoveGroup.
  It contains the service wich receives the X and Z coordinates, the arm, and configuration, moving
  the desired arm to the coordinates."""
  def __init__(self):
    super(MoveGroupElir, self).__init__()
    self.move_request_service = rospy.Service('motion_planning/MoveRequest',MoveRequest, self.go_to_pose_goal)
    
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()
    #planning scene
    scene = moveit_commander.PlanningSceneInterface()

    #Default arm configuration
    self.group_name = "f_arm"
    self.group = moveit_commander.MoveGroupCommander(self.group_name)

    #Trajectory publisher
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    #Planner from elir_inverse_kinematics
    self.custom_planner = elir_inverse_kinematics()

    # Misc variables
    self.robot = robot
    self.scene = scene

  def go_to_joint_state(self,angle_vector):
      """ Goes to the desired joint
        :param float angle_vector: Angle containing joint1 and joint2 values
        :return: 
        --``all_close`` Bool result that compares the goal and the final value using the tolerance set"""
    theta1 = angle_vector[0]
    theta2 = angle_vector[1]
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = theta1
    joint_goal[1] = theta2

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.move_group.stop()

    current_joints = self.move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.1)

  def go_to_pose_goal(self,req):
      """ Service callback wich send the group to the 
        :param MoveRequest req: Service Request variable
        :return: 
        --``result`` Bool result that compares the goal and the final value using the tolerance set"""
    #Services variables containing the x and z goal, with the specified arm configuratino
    x_goal = req.x
    z_goal = req.z
    elbow_up = req.elbow_up
    
    #Instantiates the group to be controlled
    self.group_name = req.group
    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
    #Calculates the inverse kinematics from the elir_inverse_kinematics
    joint_target = self.custom_planner.inverse_kinematics(x_goal,z_goal,elbow_up)
    #Goes to the joint state
    result = self.go_to_joint_state(joint_target)
    return result


def main():
  #Main Function that initializes the node
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('elir_move_group', anonymous=True)
  
  x = MoveGroupElir()
if __name__ == '__main__':
  main()
