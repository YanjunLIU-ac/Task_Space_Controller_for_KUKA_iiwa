%% Compute rotation error using equivalent axis represenation
% @author: yanjun liu
% @date: Aug, 2022

function rotation_error = rotationErrorByEqAxis(eulers, eulers_d)
% @param[in] eulers: current orientation in euler angles
% @param[in] eulers_d: desired orientation in euler angles
% @param[out] rotation_error: rotation error in 3-d form

axis = zeros(3, 1);

% rotation matrix
R = rotx(double(eulers(1)*180/pi)) * roty(double(eulers(2)*180/pi)) * rotz(double(eulers(3)*180/pi));
R_d = rotx(double(eulers_d(1)*180/pi)) * roty(double(eulers_d(2)*180/pi)) * rotz(double(eulers_d(3)*180/pi));
% rotation error
R_e = R * R_d';

% rotation angle
theta = acos((R_e(1,1)+R_e(2,2)+R_e(3,3)-1)/2);

% equivalent axis
if (theta ~= pi)&&(theta ~= 0)           		% theta=0: no rotaion
    axis(1) = (R(3,2)-R(2,3)) / (2 * sin(theta));
    axis(2) = (R(1,3)-R(3,1)) / (2 * sin(theta));
    axis(3) = (R(2,1)-R(1,2)) / (2 * sin(theta));
elseif theta == pi                              % theta=pi, sin(theta)=0
    axis(1) = sqrt((R(1,1)+1)/2);
    axis(2) = sqrt((R(2,2)+1)/2);
    axis(3) = sqrt((R(3,3)+1)/2);
end

% equivalent axis represenation: rotation angle * axis
rotation_error = axis * theta;