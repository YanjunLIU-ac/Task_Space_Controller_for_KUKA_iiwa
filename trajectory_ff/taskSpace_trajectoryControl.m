%% Task-space trajectory tracking for redundant robot
% @author: yanjun liu 
% @date: June, 2022
% @descriptions: 
% This scripts implements feedforward control with optional nullspace motion damper.
% Only cartesian position control is implemented.
% => tau = M*qdotdot_c + C + G;
% ==> qdotdot_c = J#(xdotdot_c - J_dot*qdot) + N*qdotdot_null;
% ==> xdotdot_c = xdotdot_d + Kx*(dx-x) + Dx*(-J*qdot), while desired trajectory is given
% => [OPTIONAL] qdotdot_null = -Dq*(-qdot)
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
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
% get the joint handles: joint 1~7
jointName = 'iiwa_joint_';
jointHandle = zeros(jointNum,1); % column vector
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
currCmdTime = vrep.simxGetLastCmdTime(clientID) / 1000;
lastCmdTime = currCmdTime;
vrep.simxSynchronousTrigger(clientID);         % every time we call this function, verp is triggered

%% DATA LOGGER
Q = []; QDOT = []; 	% joint position and velocity
DX = []; X = [];	% desired cartesian position and real cartesian position
U = [];				% joint torque
T = [];				% time elapse
ERR = []; 			% cartesian position error

%% PD CONTROLLER
% K_x = 500 * eye(3); D_x = 45 * eye(3);      % max error: 5mm
% K_x = 800 * eye(3); D_x = 56 * eye(3);      % max error: 4mm
% K_x = 1000 * eye(3); D_x = 63 * eye(3);     % max error: 3.5mm
K_x = 2000 * eye(3); D_x = 89 * eye(3);     % max error: 2.5mm
% [OPTIONAL] joint damper
D_q = 0 * eye(7);

%% SIMULATION STARTS
disp('being in loop!');
t = 0; tInit = 0;
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
	
    %% 2. set desired cartesian trajectory (position, velocity and acceleration)
    dx = 0.5 * ones(3, 1); dx(2) = 0.05 * sin(2*pi*(t-tInit));
    dxdot = [0, 0.05*2*pi*cos(2*pi*(t-tInit)), 0]';
    dxdotdot = [0, -0.05*4*pi^2*sin(2*pi*(t-tInit)), 0]';

    %% 3. integrate torque for trajectory tracking control
    % collect state variables
    q = jointPos; qdot = jointVel;
    x = eePos; % x_dot = J * qdot
    % model-based dynamics and compensation
    G = gravityTorque(lbr, q);
    C = -velocityProduct(lbr, q, qdot);
	M = massMatrix(lbr, q);
    
	%% 3-a. figure out geometric Jacobian and nullspace projection
	% Jacobian differenation, `dJ/dt = ∂J/∂q * ∂q/dt = ∂J/∂q * dq/dt = ∂(J*qdot)/∂q`
	[J, J_dot] = JacobianDerivative(lbr, q, qdot);
	transJacobian = J(1:3, :);   		% translational jacobian
    transJacobian_dot = J_dot(1:3, :);
    % NOTE THAT: Moore-Penrose of translational Jacobian in two methods
	% ***** (1) inertial-weighted generalized inverse Jacobian *****
    % % transJacobian_inv = (M \ transJacobian') / ((transJacobian / M) * transJacobian');
    % ***** (2) ordinary Moore-Penrose of translational Jacobian *****
	transJacobian_inv = transJacobian' / (transJacobian * transJacobian');
	% OPTIONAL: constraint joint movement by spring-damper system
	N = eye(7) - transJacobian_inv * transJacobian;
    
	%% 3-b. [OPTIONAL] dynamics equation in Cartesian space
    % transJacobian_inv_transpose = transJacobian_inv';
    % % Mx = transJacobian_inv_transpose * M * transJacobian_inv;
    % % Cx = transJacobian_inv_transpose * (C - M * transJacobian_inv * transJacobian_dot * qdot);
    % % Gx = transJacobian_inv_transpose * G;
	% feedforward control based on cartesian dynamics
	%% 3-c. main task: second-order inverse kinematics
    % % xdotdot_c = xdotdot_d + K_x*(dx-x) + D_x*(dxdot-transJacobian*qdot); 
    %% 3-d. formulate control law
    % % tau = transJacobian' * (Mx * xdotdot_c + Cx + Gx);

    %% NOTE 1:
    % controllers without INERTIAL-MODEL (M) are hard to obtain good
    % performance in trajectory tracking task
    % % tau = transJacobian'*(K_x*(dx-x)-D_x*transJacobian*qdot) + C + G; 
	
    %% NOTE 2:
    % Instead, FEEDFORWARD (FF) controllers are often used in such scenarios.
    % In FF controllers, desired joint position, velocity and acceleration
    % are all needed to form dynamic feedforward control signals. And a
    % PID controller is often utilized to eliminate control errors.
    % (refer to Lecture2 in Advanced Experiments for Robotic Systems)
	% joint space feedforward control
    % % C_d = -velocityProduct(lbr, dq, dqdot);
    % % G_d = gravityTorque(lbr, dq);
    % % M_d = massMatrix(lbr, dq);
    % % [J, J_dot] = JacobianDerivative(lbr, dq, dqdot);
	% % transJacobian = J(1:3, :);   % translational jacobian
    % % transJacobian_dot = J_dot(1:3, :);
    % % transJacobian_inv = (M_d \ transJacobian') / ((transJacobian / M_d) * transJacobian');
    
	%% 3-c. main task: joint-space acceleration
	% PD controller for feedback within inner-loop (second-order inverse kinematics)
    xdotdot_c = dxdotdot + K_x*(dx-x) + D_x*(dxdot-transJacobian*qdot); 
	xdotdot_main = xdotdot_c - transJacobian_dot * qdot;
	%% 3-c. [OPTIONAL] nullspace task: joint motion damper
	qdotdot_null = -D_q * qdot;
	
	%% 3-d. formulate control law based on joint-space dynamics equation
	qdotdot_c = transJacobian_inv * xdotdot_main + N * qdotdot_null;
	tau = M * qdotdot_c + C + G;
    
	%% NOTE 3: 
	% PD controller for feedback of outter-loop (outside feedforward model)
	% can also be used for trajectory control, it acts as if a virtual force is 
	% applied on the end-effector. Thus a jacobian mapping between cartesian torque
	% and joint torque is needed.
    % main task: joint-space acceleration
	% % xdotdot_c = dxdotdot;
	% % qdotdot_c = transJacobian_inv*(xdotdot_c - transJacobian_dot*qdot);
    % % virtual_pd = K_x*(dx-x) + D_x*(dxdot-transJacobian*qdot);
	% formulate control law based on joint-space dynamics equation (outter-loop)
    % % tau = M * qdotdot_c + C + G + transJacobian' * virtual_pd;

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
		vrep.simxSetJointTargetVelocity(clientID, jointHandle(i), sign(setForce)*1e10, vrep.simx_opmode_oneshot);% 决定了力的方向
		tau(i) = setForce;
		if setForce < 0
			setForce = -setForce;
		end
		vrep.simxSetJointForce(clientID, jointHandle(i),setForce , vrep.simx_opmode_oneshot);           
    end

	%% 5. data stream recording for plotting
	t = t + dt;                         % updata simulation time
    T = [T t];
	U = [U; tau']; Q = [Q; q']; QDOT = [QDOT; qdot'];
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
title('End-effector Cartesian Error of Trajectory Controller');
savefig('results\3dof\trajectory_cartpos_error.fig');

figure(2);
plot3(DX(:, 1), DX(:, 2), DX(:, 3), 'r', 'LineWidth', 1.0); hold on;
plot3(X(:, 1), X(:, 2), X(:, 3), 'b', 'LineWidth', 1.0); 
legend('Real Traj', 'Desired Traj');
xlabel('X-axis[m]'); ylabel('Y-axis[m]'); zlabel('Z-axis[m]');
zlim([0.45, 0.55]);
title('End-effector Cartesian Path of Trajectory Controller'); grid on;
savefig('results\3dof\trajectory_cartpos_path.fig');

plotJointFigures(3, T, U, "Joint Torque of Setpoint Controller", 'Torque[Nm]')
savefig('results\3dof\trajectory_joint_torque.fig');
plotJointFigures(4, T, Q, "Joint Configuration of Setpoint Controller", 'Angle[rad]')
savefig('results\3dof\trajectory_joint_config.fig');
plotJointFigures(5, T, QDOT, "Joint Velocity of Setpoint Controller", 'Velocity[rad/s]')
savefig('results\3dof\trajectory_joint_vel.fig');

rmpath('libs')