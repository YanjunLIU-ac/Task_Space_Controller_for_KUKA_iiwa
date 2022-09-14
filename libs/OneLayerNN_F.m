% @author: yanjun liu
% @date: Aug 2022
% This script constructs a neural network for robot dynamics items approximation

function [F, phi_F] = OneLayerNN_F(qdot, W_F_hat)
% @param[in] qdot: joint velocity, denoted as a column vector, 7x1
% @param[in] W_F_hat: neural net weight, 7*2x7
% @param[out] phi_F: the basis function, 7*2x1
% @param[out] F: output of the network, denoted as a column vector, 7x1

assert((size(W_F_hat, 1) == 2*7) && (size(W_F_hat, 2) == 7));

phi_F = [qdot; sign(qdot)];
F = W_F_hat' * phi_F;
end