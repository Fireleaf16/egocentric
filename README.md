# egocentric
This is for ICRA2025. There are three modes for the egocentric control: `baseline` `egocentric` `combine`. 

The `baseline` inherits the previous control method but with obstacle avoidance and with updated fuzzy control for base control. The robot end-effectors will strictly follows the user's arm movement unless it detects that the user's movement will leads to collision with existing obstacles. In that case, it will erase the direction that may cause collision but carry on other directions.(For example, if the robot end-effectors is above the task, and user is trying to grab a bottle on the table. The end-effector will stop move downward(in z direction) but will allow user to move in planar(XY directions) 

The `egocentric` is the noval control method also called `notouchpad` methodwhich prensented in the paper. It abandons the tradition key-press control. Instead, it allows the user to control the entire robot only use arm movement and head rotation. To be more specific, for the arm control, there exists a workspace for the arm which is calculated through manuverability. In which, it guarantes that no collsions with other part and no singularity. WHen the robot end-effectors are within the workspace, it strictly follows the user's arm movement. However, when the robot end-effectors hit the boundary, depending on which boundary it hits, the robot will 
 
To activate the Tiago robot, first git the repository(which should be done in the lab computer). Connect to the robot first and then open another terminal with the following command：

'''
Master
IP
cd Tiago_dual_robot
source devel/setup.bash
'''

