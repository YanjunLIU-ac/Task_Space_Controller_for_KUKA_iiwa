% @author: yanjun liu
% @date: Aug 2022
% This script constructs a neural network for robot dynamics items approximation

function [V_sigma_product, phi_V] = OneLayerNN_V(q, qdot, sigma2, W_V_hat)
% @param[in] q: joint position, denoted as a column vector, 7x1
% @param[in] qdot: joint velocity, denoted as a column vector, 7x1
% @param[in] sigma2: filtered velocity in joint space, 7x1
% @param[in] W_V_hat: neural net weight, 2*7*7*7x7
% @param[out] phi_V: the basis function, 2*7*7*7x1
% @param[out] V_sigma_product: output of the network, denoted as a column vector, 7x1

assert((size(W_V_hat, 1) == 2*7^3) && (size(W_V_hat, 2) == 7));

sinq = sin(q); cosq = cos(q);
phi_V = [kron(kron(sinq, qdot), sigma2); kron(kron(cosq, qdot), sigma2)];
V_sigma_product = W_V_hat' * phi_V;
end