# Task_Space_Controller_for_KUKA_iiwa
Example task-space controllers for KUKA LBR iiwa14 robot, including PD controller, feedforward controller, sliding mode controller, adaptive neural-network controller and nullspace impedance controller. Both position and orientation control are implemented in this project, which can be used for setpoint motion, trajectory tracking and nullspace multi-priority control. This project is fully implemented in MATLAB scripts and the simulatios run in CoppeliaSim.

## Usage
1. Open CoppeliaSim, select the scenario you wish to simulate. For example, for task-space setpoint control, you should open the scene located in `setpoint_pd/taskSpace_setpointControl.ttt`.
2. Open the MATLAB script corresponding to your simulated scene. For example, for task-space setpoint control, you should open the script located in `setpoint_pd/taskSpace_setpointControl.m` for position control (or `setpoint_pd/taskSpace_setpointControl_6dof.m` for pose control).
3. Run the MATLAB script first and then run the scene in CoppeliaSim.
* The default simulation time is 5 seconds, which can be set in the script. After simulation terminated, the script will automatically plot curves of joint torque, configuration, velocity and cartesian path, position and orientation error. A manual reset on CoppeliaSim is needed before starting a new simulation session. 

## Results
1. Setpoint PD controller (without pose control)

2. Feedforward controller for trajectory tracking (with pose control)

3. Adaptive neural-network controller for trajectory tracking (without pose control)

4. Nullspace impedance controller (with pose control)

## References
1. Slotine, Jean-Jacques E., and Weiping Li. Applied nonlinear control. Vol. 199. No. 1. Englewood Cliffs, NJ: Prentice hall, 1991.
2. Lewis, Frank L., Kai Liu, and Aydin Yesildirek. "Neural net robot controller with guaranteed tracking performance." IEEE Transactions on Neural Networks 6.3 (1995): 703-715.
3. Sadeghian, Hamid, et al. "Null-space impedance control with disturbance observer." 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.

## Acknowledgement
This project is in partial fullfillment of the requirements of "Introduction to Robotics" (Course Number. 75990052) as taught in Fall 2021, instructed by Prof. Xiang Li (Dept. of Automation, Tsinghua University). The author would like to thank Prof. Xiang Li and members of Intelligient Robotic Manipulation Lab for the basic MATLAB codes and CoppeliaSim simulation scenes.

## Disclaimer
This code may be used for learning and discussion purposes only, and the author is not responsible for any possible academic misconduct.
