% Scripts for solving derivative of Jacobian
% @author:yanjun liu
% @date: aug 2022
% @refer: 
% [1] Rhee, J‐Y., and B. Lee. "Analytical method for differentiation of robot Jacobian." Electronics Letters 53.6 (2017): 386-387.
% [2] David Busson, Richard Bearee, Adel Olabi. Task-oriented rigidity optimization for 7 DOF redundant manipulators. 20th IFAC World Congress, Jul 2017, Toulouse, France. pp.14588-14593.

function [base_Jacobian, base_Jacobian_dot] = JacobianDerivative(rigidBodyTree, jointConfig, jointVelocity)

% The following codes, jointNames are for KUKA LBR iiwa14
%% Jacobian computation: figure out base_Jacobian
% we set d0n as the end-effector here, remeber to choose corresponding T0n and d0n for specific task
% T01 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_1");
T02 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_2");
T03 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_3");
T04 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_4");
T05 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_5");
T06 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_6");
T07 = getTransform(rigidBodyTree, jointConfig, "iiwa_link_7");
T0n = getTransform(rigidBodyTree, jointConfig, "iiwa_link_ee_kuka"); 
% x denotes the No. of joint index
z0x = [T02(1:3, 3), T03(1:3, 3), T04(1:3, 3), T05(1:3, 3), T06(1:3, 3), T07(1:3, 3)];
d0x = [T02(1:3, 4), T03(1:3, 4), T04(1:3, 4), T05(1:3, 4), T06(1:3, 4), T07(1:3, 4)];

w0x = zeros(3, 6);
base_Jacobian = zeros(6, 7);
% the first column of Jacobian
w00 = [0, 0, 0]';
z00 = [0, 0, 1]';
d0n = T0n(1:3, 4);
base_Jacobian(:, 1) = [cross(z00, d0n);z00];

if nargin == 3
    % the 2nd-7th column of Jacobian
    for i = 2:7
        base_Jacobian(:, i) = [cross(z0x(:, i-1), d0n-d0x(:, i-1)); z0x(:, i-1)];
        if i-2 == 0
            w0x(:, i-1) = w00 + z00 * jointVelocity(i-1);
        else
            w0x(:, i-1) = w0x(:, i-2) + z0x(:, i-2) * jointVelocity(i-1);
        end
    end
elseif nargin == 2
    % the 2nd-7th column of Jacobian
    for i = 2:7
        base_Jacobian(:, i) = [cross(z0x(:, i-1), d0n-d0x(:, i-1)); z0x(:, i-1)];
    end
end

%% FOR DEBUG
% geometricJ = geometricJacobian(rigidBodyTree, jointConfig, "iiwa_link_ee_kuka");
% geometricJ = [geometricJ(4:6, :); geometricJ(1:3, :)];
% disp("Jacoian difference:")
% disp(sum(sum(base_Jacobian - geometricJ)));

%% Jacobian differentation: figure out base_Jacobian_dot
if nargin == 3
	z0x_dot = zeros(3, 6);
	base_Jacobian_dot = zeros(6, 7);
	beta = base_Jacobian(1:3,7) * jointVelocity(7);
	for i = 7:-1:2
		z0x_dot(:, i-1) = cross(w0x(:, i-1), z0x(:, i-1));
		alpha = zeros(3, 1);
		for j = 1:i-1
			if j-1 == 0
				alpha = alpha + cross(z00, (d0n-d0x(:,i-1))*jointVelocity(j));
			else
				alpha = alpha + cross(z0x(:, j-1), (d0n-d0x(:,i-1))*jointVelocity(j));
			end
		end
		base_Jacobian_dot(:, i) = [cross(z0x_dot(:,i-1), d0n-d0x(:, i-1))+cross(z0x(:,i-1), alpha+beta); z0x_dot(:,i-1)];
		beta = beta + base_Jacobian(1:3,i-1) * jointVelocity(i-1);
	end
	base_Jacobian_dot(:, 1) = [cross(z00, beta);zeros(3, 1)];
elseif nargin == 2
	base_Jacobian_dot = zeros(6, 7);
end