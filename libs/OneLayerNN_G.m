% @author: yanjun liu
% @date: Aug 2022
% This script constructs a neural network for robot dynamics items approximation

function [G, phi_G] = OneLayerNN_G(q, W_G_hat)
% @param[in] q: joint position, denoted as a column vector, 7x1
% @param[in] W_G_hat: neural net weight, 7*4x7
% @param[out] phi_G: the basis function, 7*4x1
% @param[out] G: output of the network, denoted as a column vector, 7x1

g = 9.81;
n_neuros = 7+7+49+49+21+21;
assert((size(W_G_hat, 1) == n_neuros) && (size(W_G_hat, 2) == 7));

sinq = sin(q); cosq = cos(q);
sincosq = sinq * cosq'; 
sincosq = sincosq(:);
cossinq = cosq * sinq';
cossinq = cossinq(:);
sinsinq = sinq * sinq';
coscosq = cosq * cosq';
sinsinq = [sinsinq(1,2:7)';sinsinq(2,3:7)';sinsinq(3,4:7)';sinsinq(4,5:7)'; ...
           sinsinq(5,6:7)';sinsinq(6,7)'];
coscosq = [coscosq(1,2:7)';coscosq(2,3:7)';coscosq(3,4:7)';coscosq(4,5:7)'; ...
           coscosq(5,6:7)';coscosq(6,7)'];
phi_G = g * [sinq; cosq; sincosq; cossinq; sinsinq; coscosq];
G = W_G_hat' * phi_G;
end