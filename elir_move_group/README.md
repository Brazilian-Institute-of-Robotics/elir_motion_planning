## Elir Move Group Package
This packages uses the MoveIt! tutorials to implement the inverse kinematics thourgh the ```elir_inverse_kinematics``` package,
implementing a custom service witch receives the coordinates and sends the calculated goal to the controllers
#Features
* ```motion_planning/MoveRequest```- This service receives the X coordinate, Z coordinate and arm configuration (elbow up/down). The position inputs can be checked by the userfrom robot visualizer in Rviz.