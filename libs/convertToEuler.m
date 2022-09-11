%% Convert rotation error to euler angles
% @author: yanjun liu
% @date: Aug, 2022

function err_euler = convertToEuler(dEuler, eeEuler)
% @param[in] dEuler: desired euler angles (rad)
% @param[in] eeEuler: current euler angles (rad)
% @param[out] error_euler: error of euler angles (rad)

err_euler = zeros(3, 1);

for i = 1:3
	if (((dEuler(i) - eeEuler(i)) < -pi) || ((dEuler(i) - eeEuler(i)) > pi))
		err_euler(i) = dEuler(i) + eeEuler(i);
	else
		err_euler(i) = dEuler(i) - eeEuler(i);
	end
end