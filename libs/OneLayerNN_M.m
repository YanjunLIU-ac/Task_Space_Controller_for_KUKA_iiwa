% @author: yanjun liu
% @date: Aug 2022
% This script constructs a neural network for robot dynamics items approximation

function [M_sigma_product, phi_M] = OneLayerNN_M(q, sigma1, W_M_hat)
% @param[in] q: joint position, denoted as a column vector, 7x1
% @param[in] sigma1: filtered acceleration in joint-space, 7x1
% @param[in] W_M_hat: neural net weight, 3*7*7x7
% @param[out] phi_M: basis function, 3*7*7x1
% @param[out] M_sigma_product: output of the network, denoted as a 7x1 column vector

assert((size(W_M_hat, 1) == 3*7^2) && (size(W_M_hat, 2) == 7));

sinq = sin(q); cosq = cos(q);
phi_M = [kron(q, sigma1); kron(sinq, sigma1); kron(cosq, sigma1)];
M_sigma_product = W_M_hat' * phi_M;
end