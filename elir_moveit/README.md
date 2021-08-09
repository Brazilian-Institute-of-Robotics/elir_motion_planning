## Elir MoveIt! Package
This package was generated using the MoveIt! Setup Assistant Tool.
The robot descrpition file is contained in the elir_description package,
contained in the elir_simulation repository

#Features
* This package is already compatibilized with the real robot, the compatibilization procedure is the same as with the simulation, to control the robot with
MoveIt! just run:
```
roslaunch elir_moveit elir_planning_execution.launch
```
It is possible to control de f_arm and the b_arm with the interactive marker

#MoveIt! Configuration

The package already comes configured, but in order to configure it in your robot, do the following instructions.
Create one controlers.yaml file and paste it on the config folder, this file points to move it wich one is the action server declared in gazebo, the example of one arm controller is:
```
controller_manager_ns: controller_manager
controller_list:
  - name: robot/f_arm_trajectory_controller/
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint1_f
      - joint2_f

```
Then, one launch file is necessary to load this configuration, generally it is already created inside of the move it generated package, under the name of your_robot_moveit_controller_manager.launch.xml.It starts the MoveItSimpleControllerManager and loads the joint trajectory controller interface defined inside controllers.yaml.

```
<launch>
	<!-- Set the param that trajectory_execution_manager needs to find the controller plugin -->
	<arg name="moveit_controller_manager" default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
	<param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>

	<!-- load controller_list -->
	<arg name="use_controller_manager" default="true" />
	<param name="use_controller_manager" value="$(arg use_controller_manager)" />

	<!-- Load joint controller configurations from YAML file to parameter server -->
	<rosparam file="$(find moveit_with_gazebo)/config/controllers.yaml"/>
</launch>
```

Now, create one moveit_planning_execution.launch file, that loads the planning execution.

```
<launch>
 # The planning and execution components of MoveIt! configured to 
 # publish the current configuration of the robot (simulated or real)
 # and the current state of the world as seen by the planner
 <include file="$(find moveit_with_gazebo)/launch/move_group.launch">
  <arg name="publish_monitored_planning_scene" value="true" />
 </include>
 # The visualization component of MoveIt!
 <include file="$(find moveit_with_gazebo)/launch/moveit_rviz.launch"/>
</launch>
```
## Configuring the Rviz
When you first initalize RVIZ , it doesn't come configured with the motion planning plugin, so click on add, and add the motion planning plugin,
and change the fixed frame at global options to base_link.