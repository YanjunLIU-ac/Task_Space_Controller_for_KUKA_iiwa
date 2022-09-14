%% Neural-net controller for trajectory tracking on redundant robot
% @author: yanjun liu 
% @date: June, 2022
% @description:
% This script implements trajectory tracking based on partitioned neural-net controller on KUKA LBR iiwa14, i,e,.
% => tau = M_hat + C_hat + G_hat + Kr * r
% where approximation M_hat, C_hat and G_hat are defined as:
% ==> M_hat = W_M' * phi_M(sigma1, q), C_hat = W_C' * phi_C(sigma2, q, qdot), G are compensated real-time.
% where the update law of neural net weight is defined as: (improved weight tuning)
% ==> W_dot = F * phi * r' - k * F * norm(r) * W; W = W + dt * W_dot;
% @NOTE THAT: without gravity compensation, the network is hard to obtain desired performance.
% @references: Neural Net Robot Controller with Guaranteed Tracking
clear; clc; close all;
addpath("..\..\libs");

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
% get the ee handle
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
[~, eeEuler] = vrep.simxGetObjectOrientation(clientID, eeHandle, -1, vrep.simx_opmode_streaming);
eePos = eePos';
eeEuler = eeEuler';
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
U_model = [];		% model-based torque

%% NEURAL NETWORK CONTROLLER
n_input_M = 3*7^2;
n_input_V = 2*7^3;
n_input_G = 7*2+7*7*2+7*6;
n_input_F = 2*7;
% for neural network: weight for neural network layer
W_M = zeros(n_input_M, 7);
W_V = zeros(n_input_V, 7);		    
W_G = zeros(n_input_G, 7);
W_F = zeros(n_input_F, 7);
% for neural network: any constant matrix for weight tuning
F_value = 0.01;
k_value = 0.01;
F_M = F_value * eye(n_input_M);
F_V = F_value * eye(n_input_V);
F_G = F_value * eye(n_input_G);
F_F = F_value * eye(n_input_F);
k_M = k_value * eye(n_input_M);
k_V = k_value * eye(n_input_V);
k_G = k_value * eye(n_input_G);
k_F = k_value * eye(n_input_F);
% for sliding mode vector
K_r = 1;
K_x1 = 100;
K_x2 = 100;
D_x2 = 5;

%% LOAD PRE-TRAINED WEIGHTS
% load('data\W_M.mat');
% load('data\W_V.mat');
load('data\W_G.mat');
% load('data\W_F.mat');

%% SIMULATION STARTS
disp('being in loop!');
t = 0; tInit = 0;
while ((vrep.simxGetConnectionId(clientID) ~= -1) && (t<10))  % vrep connection is still active    
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
    [~, eeEuler] = vrep.simxGetObjectOrientation(clientID, eeHandle, -1, vrep.simx_opmode_buffer);
    eePos = eePos';
    eeEuler = eeEuler';
	
    %% 2. set desired cartesian trajectory (position, velocity and acceleration)
    dx = 0.5 * ones(3, 1); dx(2) = 0.05 * sin(2*pi*(t-tInit));
    dxdot = [0, 0.05*2*pi*cos(2*pi*(t-tInit)), 0]';
    dxdotdot = [0, -0.05*4*pi^2*sin(2*pi*(t-tInit)), 0]';
	if ~exist('last_qdot', 'var')
		last_qdot = zeros(7, 1);
	end

    %% 3. integrate torque for trajectory tracking control
    % collect state variables
    q = jointPos; qdot = jointVel;
    x = eePos; % x_dot = J * qdot
    % model-based dynamics and compensation
    % % G = gravityTorque(lbr, q);
    % % C = -velocityProduct(lbr, q, qdot);
	% % M = massMatrix(lbr, q);
	
    %% 3-a. figure out geometric Jacobian and nullspace projection
	% Jacobian differenation, `dJ/dt = ∂J/∂q * ∂q/dt = ∂J/∂q * dq/dt = ∂(J*qdot)/∂q`
	[J, J_dot] = JacobianDerivative(lbr, q, qdot);
	transJacobian = J(1:3, :);   % translational jacobian
    transJacobian_dot = J_dot(1:3, :);
    % NOTE THAT: Moore-Penrose of translational Jacobian in two methods
	% ***** (1) inertial-weighted generalized inverse Jacob *****
    % transJacobian_inv = (M \ transJacobian') / ((transJacobian / M) * transJacobian');
    % ***** (2) ordinary Moore-Penrose of translational Jacobian *****
	transJacobian_inv = transJacobian' / (transJacobian * transJacobian');

    %% 3-b. construct partitioned neural network approximation
	% sliding mode vector
	r = transJacobian_inv * (dxdot + K_x1*(dx-x)) - qdot;
	% neural input
	xdot_c = dxdot + K_x1 * (dx-x);
	xdotdot_c = dxdotdot + D_x2 * (dxdot-transJacobian*qdot) + K_x2 * (dx-x);
	sigma2 = transJacobian_inv * xdot_c;
	sigma1 = transJacobian_inv * (xdotdot_c - transJacobian_dot*transJacobian_inv*dxdot);
	% neural network propagation
	[M_hat, Phi_M] = OneLayerNN_M(q, sigma1, W_M);
	[V_hat, Phi_V] = OneLayerNN_V(q, qdot, sigma2, W_V);
	[G_hat, Phi_G] = OneLayerNN_G(q, W_G);
	[F_hat, Phi_F] = OneLayerNN_F(qdot, W_F);
	
	%% 3-c. update neural network weight
	% weight update law
	W_M_dot = F_M * Phi_M * r' - k_M * F_M * norm(r) * W_M;
	W_V_dot = F_V * Phi_V * r' - k_V * F_V * norm(r) * W_V;
	% % W_G_dot = F_G * Phi_G * r' - k_G * F_G * norm(r) * W_G;
	W_F_dot = F_F * Phi_F * r' - k_F * F_F * norm(r) * W_F;
	% update neural network weight
	W_M = W_M + W_M_dot * dt;
	W_V = W_V + W_V_dot * dt;
	% % W_G = W_G + W_G_dot * dt;
	W_F = W_F + W_F_dot * dt;

    %% 3-d. formulate control law
    % neural network approximation
	f = M_hat + V_hat + F_hat + G_hat;
	% control law
	tau = f + K_r * r;

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
    t = t + dt;                         % updata simulation time
	T = [T t];
	X = [X; x']; DX = [DX; dx']; 
    ERR = [ERR;(dx-x)'];
	U = [U; tau']; 
	computed_qdotdot = (qdot - last_qdot)/dt;
	U_model = [U_model; inverseDynamics(lbr, q, qdot, computed_qdotdot)'];
    disptime = sprintf('Simulation Time: %f s', t); 
	disp(disptime);

    %% 6. update vrep(the server side) matlab is client
	vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);     % MAKE SURE connection is still on
    lastCmdTime = currCmdTime;
    last_qdot = qdot;
end
vrep.simxFinish(-1);  % close the link
vrep.delete();        % destroy the 'vrep' class

%% PLOT FIGURES
figure(1); hold on;
plot(T, ERR(:, 1), 'r', 'LineWidth', 1.0);
plot(T, ERR(:, 2), 'g', 'LineWidth', 1.0);
plot(T, ERR(:, 3), 'b', 'LineWidth', 1.0);
legend('X-axis', 'Y-axis', 'Z-axis');
xlabel('Time Elapse[s]');
ylabel('Cartesian Error[m]');
title({'End-effector Position Error'; 'of Neural Net Trajectory Controller'});
savefig('results\improved_tuning\trajnn_cartpos_error.fig')

figure(2);
plot3(DX(:, 1), DX(:, 2), DX(:, 3), 'r', 'LineWidth', 1.0); hold on;
plot3(X(:, 1), X(:, 2), X(:, 3), 'b', 'LineWidth', 1.0); 
legend('Real Traj', 'Desired Traj');
xlabel('X-axis[m]'); ylabel('Y-axis[m]'); zlabel('Z-axis[m]');
zlim([0.45, 0.55]);
title('End-effector Cartesian Path of Neural Net Trajectory Controller'); grid on;
savefig('results\improved_tuning\trajnn_cartpos_path.fig');

plotJointFiguresComp(3, T, U_model, U, ["Joint Torque of Neural Net Trajectory Controller"; "Red: Model-based torque, Blue: NN Approximation"], 'Torque[Nm]')
savefig('results\improved_tuning\trajnn_joint_torque.fig');
plotCartFiguresComp(4, T, DX, X, ["Cartesian Positon of Neural Net Trajectory Controller"; "Red: Model-based torque, Blue: NN Approximation"], 'Position[m]')
savefig('results\improved_tuning\trajnn_cartpos_comp.fig');

rmpath("..\..\libs");