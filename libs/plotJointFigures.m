%% Plot curves per joint in a figure
% @author: yanjun liu
% @date: Aug, 2022

function plotJointFigures(fig_no, T, DATA, sgtitle_name, ylabel_name)
% @param[in] fig_no: number of figure handle
% @param[in] sgtitle_name: total name of figure
% @param[in] ylabel_name: name of y-axis label

figure(fig_no); sgtitle(sgtitle_name)
subplot(2, 4, 1); plot(T, DATA(:, 1), 'LineWidth', 0.5); ylabel(ylabel_name); legend('Joint 1');
subplot(2, 4, 2); plot(T, DATA(:, 2), 'LineWidth', 0.5); legend('Joint 2');
subplot(2, 4, 3); plot(T, DATA(:, 3), 'LineWidth', 0.5); legend('Joint 3');
subplot(2, 4, 4); plot(T, DATA(:, 4), 'LineWidth', 0.5); xlabel('Time Elapsed[s]'); legend('Joint 4');
subplot(2, 4, 5); plot(T, DATA(:, 5), 'LineWidth', 0.5); ylabel(ylabel_name); xlabel('Time Elapsed[s]'); legend('Joint 5');
subplot(2, 4, 6); plot(T, DATA(:, 6), 'LineWidth', 0.5); xlabel('Time Elapsed[s]'); legend('Joint 6');
subplot(2, 4, 7); plot(T, DATA(:, 7), 'LineWidth', 0.5); xlabel('Time Elapsed[s]'); legend('Joint 7');