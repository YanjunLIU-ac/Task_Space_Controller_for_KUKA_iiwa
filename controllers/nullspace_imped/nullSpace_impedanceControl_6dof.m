%% Nullspace setpoint control of redundant robot
% @author: yanjun liu 
% @date: June, 2022
% @descriptions: 
% This script implements feedforward cartesian control with nullspace PD-controller on KUKA LBR iiwa14, i,e,.
% => tau = M*qdotdot_c + C + G;
% ==> qdotdot_c = J#(xdotdot_c - J_dot*qdot) + N*qdotdot_null;
% ==> xdotdot_c = xdotdot_d + Kx*(dx-x) + Dx*(-J*qdot)
% ==> qdotdot_null = Kq*(dq-q) + Dq*(-qdot)
% @reference: "Nullspace Impedance Control with Disturbance Observer"
% @NOTE THAT the gripper in this scene is static.
clc, clear, close all;
addpath('..\..\libs');

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
% set the simulation time step
tstep = 0.005;  % 5ms per simulation step
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot);
% open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID, true);

%% SIMULATION INITIALIZATION
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
% get the joint 1-7 handles
jointName = 'iiwa_joint_';
jointHandle = zeros(jointNum, 1); % column vector
for i = 1:jointNum 
    [res, returnHandle] = vrep.simxGetObjectHandle(clientID, [jointName, int2str(i)], vrep.simx_opmode_blocking);
    if res == 0
        jointHandle(i) = returnHandle;
    else
        fprintf('can not get the handle of joint %d!!!\n',i);
    end
end
% get the end-effector handle (ee_kuka_visual)
[res, eeHandle] = vrep.simxGetObjectHandle(clientID, 'iiwa_link_ee_kuka_visual', vrep.simx_opmode_blocking);
if res == 0
    jointHandle(i) = returnHandle;
else
    fprintf('can not get the handle of ee %d!!!\n',i);
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
currCmdTime = vrep.simxGetLastCmdTime(clientID) / 1000;
lastCmdTime = currCmdTime;
vrep.simxSynchronousTrigger(clientID);         % every time we call this function, verp is triggered

%% DATA LOGGER
Q = []; QDOT = []; 	% joint position and velocity
DX = []; X = [];	% desired cartesian position and real cartesian position
U = [];				% joint torque
T = [];				% time elapse
cartERROR = [];		% cartesian position error
jntERROR = [];		% joint-space configuration error
VFORCE = [];        % virtual force exerted on robot

%%  PD CONTROLLER
K_x = [5000*eye(3), zeros(3); zeros(3), 5000*eye(3)];
D_x = [141*eye(3), zeros(3); zeros(3), 141*eye(3)];
K_q = 100 * eye(7); D_q = 1 * eye(7); 

%% SIMULATION STARTS
disp('being in loop!');
t = 0; tPerturb = 0.1;
dxdotdot = zeros(3, 1); dxdot = zeros(3, 1);	% desired cartesian velocity and acceleration (set to 0)
dqdotdot = zeros(7, 1);	dqdot = zeros(7, 1);	% desired joint-space velocity and acceleration (set to 0)
while ((vrep.simxGetConnectionId(clientID) ~= -1) && (t<3))  % vrep connection is still active
    %% 0. time update
    currCmdTime = vrep.simxGetLastCmdTime(clientID) / 1000;
    dt = currCmdTime-lastCmdTime;              % simulation step, unit: s 

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

    %% 2. set desired configuration (right, neural and left)
	% NOTE THAT: nullspace motion should be subjected to kinematics, i.e., x=FKINE(q)
    if ~exist('dEuler', 'var')
		dEuler = eeEuler;
    end
    if ~exist('nullspace_q_desired', 'var')
        nullspace_q_desired = jointPos;
    end
    if ~exist('dx', 'var')
        dx = eePos;
    end

    %% ***** APPLY VIRTUAL EXTERNAL FORCE *****
    if ((t < tPerturb) || (t > tPerturb + 2))
		perturb = 0;
    else
		perturb = 100*sin(2*pi*(t-tPerturb));
    end
    tau_e = 1 * [-perturb, perturb, 0, 0, 0, -500, 0]';

    %% 3. integrate torque for nullspace setpoint control
    % collect state variables
    q = jointPos; qdot = jointVel;
	x = eePos; % x_dot = J * qdot
    % model-based dynamics and compensation
    G = gravityTorque(lbr, q);
    C = -velocityProduct(lbr, q, qdot);
    M = massMatrix(lbr, q);
    
    %% 3-a. figure out geometric Jacobian and nullspace projection
	% Jacobian differenation, `dJ/dt = ∂J/∂q * ∂q/dt = ∂J/∂q * dq/dt = ∂(J*qdot)/∂q`
    [geometryJacobian, geometryJacobian_dot] = JacobianDerivative(lbr, q, qdot);
	% NOTE THAT: Moore-Penrose of translational Jacobian in two methods
	% ***** (1) inertial-weighted generalized inverse Jacob *****
    geometryJacobian_inv = (M \ geometryJacobian') / ((geometryJacobian / M) * geometryJacobian');
    % ***** (2) ordinary Moore-Penrose of translational Jacobian *****
	% geometryJacobian_inv = geometryJacobian' / (geometryJacobian * geometryJacobian');
    % nullspace projector matrix
    N = eye(7) - geometryJacobian_inv * geometryJacobian;
	
    %% 3-b. main task (xdotdot_d=0, xdot_d=0)
    e = [dx-x; rotationErrorByEqAxis(dEuler, eeEuler)];
	xdotdot_c = [dxdotdot;zeros(3,1)] + K_x*e + D_x*([dxdot;zeros(3,1)]-geometryJacobian*qdot);
    xdotdot_main = xdotdot_c - geometryJacobian_dot*qdot;
	
    %% 3-c. nullspace task (mass-spring-damper system, second-order system)
    Bd = 7 * eye(7); Kd = 9 * eye(7);
    qdotdot_null = dqdotdot + (5 * M)\(Bd*(dqdot-qdot)+Kd*(nullspace_q_desired-q)+tau_e);

    %% 3-d. integrate main and nullspace task to formulate control law
    qdotdot_c = geometryJacobian_inv * xdotdot_main + N * qdotdot_null;
    % feedforward control scheme
    tau = M * qdotdot_c + C + G;

    % the tau(7) is set to 0. since its mass is too small for torque control. otherwise it will induce instability
    tau(7) = 0;

    %% 4. set the torque in vrep way
    for i = 1:jointNum
        if tau(i) < -forceLimit
            setForce = -forceLimit;
        elseif tau(i) > forceLimit
            setForce = forceLimit;
        else
            setForce = tau(i);  % set a trememdous large velocity for the screwy operation of the vrep torque control implementaion
        end
        vrep.simxSetJointTargetVelocity(clientID, jointHandle(i), sign(setForce)*1e10, vrep.simx_opmode_oneshot);
        tau(i) = setForce;
        if setForce < 0
            setForce = -setForce;
        end
        vrep.simxSetJointForce(clientID, jointHandle(i), setForce, vrep.simx_opmode_oneshot);           
    end

    %% 5. data stream recording for plotting
    t = t + dt;     				% updata simulation time, unit:s
    T = [T t];
    Q = [Q; q']; QDOT = [QDOT; qdot']; 
    U = [U; tau']; VFORCE = [VFORCE; tau_e'];
    cartERROR = [cartERROR; [dx-x;convertToEuler(dEuler, eeEuler)]'];
    jntERROR = [jntERROR; (nullspace_q_desired-q)'];
	disptime = sprintf('Simulation Time: %f s', t);
    disp(disptime);

    %% 6. update vrep(the server side) matlab is client
	vrep.simxSynchronousTrigger(clientID);
    vrep.simxGetPingTime(clientID);		% MAKE SURE connection is still on
    lastCmdTime = currCmdTime;
    dx = dx + dxdot * dt;   		% update command position on desired trajectroy
	dxdot = dxdot + dt * dxdotdot;  % update command velocity on desired trajectory
end
vrep.simxFinish(-1);  % close the link
vrep.delete();        % destroy the 'vrep' class

%% PLOT Figures
figure(1); hold on;
plot(T, cartERROR(:, 1), 'r', 'LineWidth', 1.0);
plot(T, cartERROR(:, 2), 'g', 'LineWidth', 1.0);
plot(T, cartERROR(:, 3), 'b', 'LineWidth', 1.0);
xlabel('Time Elapsed[s]');
ylabel('Cartesian Error[m]');
legend('X-axis', 'Y-axis', 'Z-axis');
title({'End-Effector Position Error'; 'of Nullspace Impedance Controller'});
savefig('results\6dof\nullSpace_imped6_cartpos_error.fig');

figure(2); hold on;
plot(T, cartERROR(:, 4), 'r', 'LineWidth', 1.0);
plot(T, cartERROR(:, 5), 'g', 'LineWidth', 1.0);
plot(T, cartERROR(:, 6), 'b', 'LineWidth', 1.0);
xlabel('Time Elapsed[s]');
ylabel('Orientation Error[rad]');
legend('X-axis', 'Y-axis', 'Z-axis');
title({'End-Effector Orientation Error'; 'of Nullspace Impedance Controller'});
savefig('results\6dof\nullSpace_imped6_cartrot_error.fig');

plotJointFigures(3, T, VFORCE, 'Joint-space External Virtual Force to Drive Motion', 'Torque[Nm]')
savefig('results\6dof\nullSpace_imped6_joint_vforce.fig');
plotJointFigures(4, T, U, "Joint Torque of Nullspace Impedance Controller", 'Torque[Nm]')
savefig('results\6dof\nullSpace_imped6_joint_torque.fig');
plotJointFigures(5, T, Q, "Joint Configuration of Nullspace Impedance Controller", 'Angle[rad]')
savefig('results\6dof\nullSpace_imped6_joint_config.fig');
plotJointFigures(6, T, QDOT, "Joint Velocity of Nullspace Impedance Controller", 'Velocity[rad/s]')
savefig('results\6dof\nullSpace_imped6_joint_vel.fig');
plotJointFigures(7, T, jntERROR, "Joint Error of Nullspace Impedance Controller", 'Angle[rad/s]')
savefig('results\6dof\nullSpace_imped6_joint_error.fig');

rmpath('..\..\libs')