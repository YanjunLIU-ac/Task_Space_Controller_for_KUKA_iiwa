% @author: yanjun liu 
% @date: Aug 2022
% @description:
% This script is to validate analytical solution of Jacobian differentation with numeric one.
% i.e., we seek to compare the result of func JacobianDerivative with the following results:
% dJ/dt = ∂J/∂q * ∂q/dt = ∂J/∂q * dq/dt = ∂(J*qdot)/∂q
% ==> dJ(m,n)/dt = \SUM_{k=1}^{7}{qdot_k*∂J(m,n)/∂q_k}
% where parital derivative is calculated with two-point formula consistent with reference[1].
% ==> ∂J(q1,...,q7)/∂q1 = (J(q1+h,...,q7)-J(q1-h,...,q7))/(2*h)
% @reference: 
% [1] Rhee, J‐Y., and B. Lee. "Analytical method for differentiation of robot Jacobian." Electronics Letters 53.6 (2017): 386-387.

clc, clear;
%%  Create the robot (KUKA LBR iiwa14 as an example)
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'column';
lbr.Gravity = [0 0 -9.81];
forceLimit = 5000;
displayOn = false;
jointNum = 7;

%% Initialize robot with sinusoidal joint positions and velocities
t = 0; dt = 0.01;	% sample every 10ms with duration of 5s
q = lbr.randomConfiguration;
jointConfig = [sin(q(1)), sin(q(2)), sin(q(3)), sin(q(4)), sin(q(5)), sin(q(6)), sin(q(7))]';
jointVelocity = 0.01*[sin(q(1)), sin(q(2)), sin(q(3)), sin(q(4)), sin(q(5)), sin(q(6)), sin(q(7))]';

%% Initial logger for difference computation
DIFF = []; T = [];
NUM_T = 0; ANA_T = 0;

%% Start simulation
iters = 0;
while (t < 1)
    iters = iters + 1;
	% numeric solution
	delta_q = 1e-8 * eye(7);	% minor displacement: h=1e-8
	numJacob_dot = zeros(6, 7);
	
	t0 = tic;
	for i = 1:6
		for j = 1:7
			% for each joint
			numJacob_dot(i,j) = 0;
			for k = 1:7		% computation for j-th joint
				[J_plus, ~] = JacobianDerivative(lbr, jointConfig+delta_q(:,k));
				[J_minus, ~] = JacobianDerivative(lbr, jointConfig-delta_q(:,k));
				numJacob_dot(i,j) = numJacob_dot(i,j) + (J_plus(i,j)-J_minus(i,j))/(2*1e-8)*jointVelocity(k);
			end
		end
	end
	NUM_T = NUM_T + toc(t0);
	
	% analytical solution
	t1 = tic;
	[~, Jacob_dot] = JacobianDerivative(lbr, jointConfig, jointVelocity);
	ANA_T = ANA_T + toc(t1);

	t = t + dt;		% update time
	T = [T; t];
    jointConfig = jointConfig + dt * jointVelocity;     % update joint position
    diff = numJacob_dot-Jacob_dot;
	DIFF = [DIFF;diff(1,:), diff(2,:), diff(3,:), diff(4,:), diff(5,:), diff(6,:)];
end

%% Plot figures
figure(1);
for i = 1:6
	for j = 1:7
		subplot(6, 7, 7*(i-1)+j);
		xlabel('time(s)');
		ylabel('dJ/dt');
		title(["(", num2str(i), ",", num2str(j), ")th component"]);
		plot(T, DIFF(:, 7*(i-1)+j), 'Color', [0,0,0.50196]);
        ylim([-1e-7, 1e-7]);
	end
end
sgtitle('Difference between numerical values and analytical values');
disp('Every time for analytical method:')
disp(ANA_T/iters);
disp('Every time for numeric method:')
disp(NUM_T/iters);