## Elir Inverse Kinematics Package
This package is used to calculate the Inverse kinematics for the Elir Arms.
It was configurated considering the end effector of the robot in the limit of 
the tracion unit, marked as a black box in the URDF visualization.

It was configured to be imported as a function in other packages, like in the 
elir_move_group, that creates the calls for services to move the robot.

#Features
* The code src/kinematics_solver/solver.py contains the solver for the inverse kinematics,
it considers the arm as a planar arm, wich you specify the x and z coordinates, according to 
the rviz referencial, and the configuration of the arm, that can be elbow up or elbow down.