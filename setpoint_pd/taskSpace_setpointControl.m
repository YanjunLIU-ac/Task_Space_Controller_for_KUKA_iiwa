%% Task-space setpoint control for redundant manipulator
% @author: yanjun liu 
% @date: June, 2022
% @descriptions: 
% This script implements PD-based setpoint controller on KUKA LBR iiwa14.
% Only for cartesian position control and joint damper included.
% ==> tau = J_inv * (Kp*x_tilde + Kd*xdot_tilde) + Dj * qdot_tilde + G;
clear; clc; close all;
addpath('libs');

%% LOAD ROBOT
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'column';
lbr.Gravity = [0 0 -9.81];
forceLimit = 5000;
displayOn = false;
jointNum = 7;

%% CONNECT TO VREP
vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
% close all the potential link
vrep.simxFinish(-1);
% wait for connecting vrep, detect every 0.2s
while true
    clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
    if clientID > -1 
        break;
    else
        pause(0.2);
        disp('please run the simulation on vrep...')
    end
end
disp('Connection success!')
% set the simulation time step: 5ms per simulation step
tstep = 0.005;
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot);
% open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID, true);

%% SIMULATION INITIALIZATION
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
% get the joint handles: joint 1~7
jointName = 'iiwa_joint_';
jointHandle = zeros(jointNum, 1); % column vector
for i = 1:jointNum 
    [res, returnHandle] = vrep.simxGetObjectHandle(clientID, [jointName, int2str(i)], vrep.simx_opmode_blocking);
    if res == 0
        jointHandle(i) = returnHandle;
    else
        fprintf('can not get the handle of joint %d!!!\n', i);
    end
end
% get the end-effector handle (ee_kuka_visual)
[res, eeHandle] = vrep.simxGetObjectHandle(clientID, 'iiwa_link_ee_kuka_visual', vrep.simx_opmode_blocking);
if res == 0
    jointHandle(i) = returnHandle;
else
    fprintf('can not get the handle of ee %d!!!\n', i);
end
vrep.simxSynchronousTrigger(clientID);
disp('Handles available!')

%% SET DATASTREAM
% NOTE: 
% datastream mode set to simx_opmode_streaming for the first call to read
% the joints' configuration, afterwards set to simx_opmode_buffer
jointPos = zeros(jointNum, 1);	% joint position
jointVel = zeros(jointNum, 1);	% joint velocity
jointTau = zeros(jointNum, 1); 	% joint torque
for i = 1:jointNum
    [~, jointPos(i)] = vrep.simxGetJointPosition(clientID, jointHandle(i), vrep.simx_opmode_streaming);
    [~, jointVel(i)] = vrep.simxGetObjectFloatParameter(clientID, jointHandle(i), 2012, vrep.simx_opmode_streaming);
    [~, jointTau(i)] = vrep.simxGetJointForce(clientID, jointHandle(i), vrep.simx_opmode_streaming);
end
% end effector position (x-y-z coordinates) and its orientation (alpha,beta,gamma)
[~, eePos] = vrep.simxGetObjectPosition(clientID, eeHandle, -1, vrep.simx_opmode_streaming);
eePos = eePos';
% get simulation time
currCmdTime = vrep.simxGetLastCmdTime(clientID);
lastCmdTime = currCmdTime / 1000;
vrep.simxSynchronousTrigger(clientID);         % every time we call this function, verp is triggered

%% DATA LOGGER
Q = []; QDOT = []; 	% joint position and velocity
DX = []; X = [];	% desired cartesian position and real cartesian position
U = [];				% joint torque
T = [];				% time elapse
ERR = [];			% cartesian position error

%% PD CONTROLLER
K_x = 80 * eye(3); D_x = 20 * eye(3);
D_q = 1 * eye(7);

%% SIMULATION STARTS
disp('being in loop!');
t = 0;
while ((vrep.simxGetConnectionId(clientID) ~= -1) && (t < 5))  % vrep connection is still active    
    %% 0. time update
    currCmdTime = vrep.simxGetLastCmdTime(clientID) / 1000;
    dt = currCmdTime - lastCmdTime;               % simulation step, unit: s 
    
    %% 1. read state variables from sensors
	% joint position and velocity
    for i = 1:jointNum
        [~, jointPos(i)] = vrep.simxGetJointPosition(clientID, jointHandle(i), vrep.simx_opmode_buffer);
        [~, jointVel(i)] = vrep.simxGetObjectFloatParameter(clientID, jointHandle(i), 2012, vrep.simx_opmode_buffer);
    end
    % end-effector's cartesian position and orientation
    [~, eePos] = vrep.simxGetObjectPosition(clientID, eeHandle, -1, vrep.simx_opmode_buffer);
    eePos = eePos';
	
    %% 2. set desired x 
	% only once, from [0.5, -0.2, 0.5] to [0.4, -0.3, 0.4]
	% no orientation control included
    if ~exist('dx','var')
        dx = eePos - 0.1;
    end

    %% 3. integrate torque for setpoint control
	% collect state variables
    q = jointPos; qdot = jointVel;
    x = eePos; % x_dot = J * qdot
	% model-based dynamics and compensation
    G = gravityTorque(lbr, q);
    C = -velocityProduct(lbr, q, qdot);
	% model-based geometric jacobian
    J = geometricJacobian(lbr, q, "iiwa_link_ee_kuka");
	% translational jacobian: note that 4-6 for translation, 1-3 for rotation
    transJacobian = J(4:6, :);   
	
	%% PD-based setpoint controller, with joint motion damper
	% NOTE:
	% With joint motion (velocity) damper, robot can reach desired position with less configuration movement.
	% Otherwise, robot joints may performance very large and dramatic air flare during setpoint motion.
    tau = transJacobian'*(K_x*(dx-x)-D_x*transJacobian*qdot) - D_q*qdot + G; % + C;
    
	% the tau(7) is set to 0. since its mass is too small for torque control. otherwise it will induce instability
	tau(7) = 0;

    %% 4. set the torque in vrep way
    for i = 1:jointNum
		if tau(i) < -forceLimit
			setForce = -forceLimit;
		elseif tau(i) > forceLimit
			setForce = +forceLimit;
		else
			setForce = tau(i); % set a trememdous large velocity for the screwy operation of the vrep torque control implementaion
		end
		vrep.simxSetJointTargetVelocity(clientID, jointHandle(i), sign(setForce)*1e10, vrep.simx_opmode_oneshot);
		tau(i) = setForce;
		if setForce < 0
			setForce = -setForce;
		end
		vrep.simxSetJointForce(clientID, jointHandle(i),setForce , vrep.simx_opmode_oneshot);           
    end

    %% 5. data stream recording for plotting
    t = t + dt;     					% updata simulation time
    T = [T t];
    U = [U; tau'];
	Q = [Q; q']; QDOT = [QDOT; qdot']; 
	X = [X; x']; DX = [DX; dx']; ERR = [ERR;(dx-x)'];
    disptime = sprintf('Simulation Time: %f s', t);
    disp(disptime);

    %% 6. update vrep(the server side) matlab is client
	vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);     % MAKE SURE connection is still on
    lastCmdTime = currCmdTime;
end
vrep.simxFinish(-1);  % close the link
vrep.delete();        % destroy the 'vrep' class

%% PLOT ERROR
figure(1); hold on;
plot(T, ERR(:, 1), 'r', 'LineWidth', 1.0);
plot(T, ERR(:, 2), 'g', 'LineWidth', 1.0);
plot(T, ERR(:, 3), 'b', 'LineWidth', 1.0);
legend('X-axis', 'Y-axis', 'Z-axis');
xlabel('Time Elapsed[s]');
ylabel('Cartesian Error[m]');
title('End-effector Cartesian Error of Setpoint Controller');
savefig('results\3dof\setpoint_cartpos_error.fig');

figure(2);
scatter3(dx(1), dx(2), dx(3), 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'LineWidth', 0.8); hold on; 
scatter3(0.5, -0.2, 0.5, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g', 'LineWidth', 0.8);
plot3(X(:, 1), X(:, 2), X(:, 3), 'b', 'LineWidth', 1.0); 
legend("Desired Setpoint", "Initial Setpoint", "Real Traj");
xlabel('X-axis[m]'); ylabel('Y-axis[m]'); zlabel('Z-axis[m]');
title('End-effector Cartesian Path of Setpoint Controller');
savefig('results\3dof\setpoint_cartpos_path.fig');

plotJointFigures(3, T, U, "Joint Torque of Setpoint Controller", 'Torque[Nm]')
savefig('results\3dof\setpoint_joint_torque.fig');
plotJointFigures(4, T, Q, "Joint Configuration of Setpoint Controller", 'Angle[rad]')
savefig('results\3dof\setpoint_joint_config.fig');
plotJointFigures(5, T, QDOT, "Joint Velocity of Setpoint Controller", 'Velocity[rad/s]')
savefig('results\3dof\setpoint_joint_vel.fig');

rmpath('libs');