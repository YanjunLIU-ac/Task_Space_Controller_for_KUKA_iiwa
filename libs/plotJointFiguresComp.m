%% Plot two curves per joint for comparison in a figure
% @author: yanjun liu
% @date: Aug, 2022

function plotJointFiguresComp(fig_no, T, DATA1, DATA2, sgtitle_name, ylabel_name)
% @param[in] fig_no: number of figure handle
% @param[in] sgtitle_name: total name of figure
% @param[in] ylabel_name: name of y-axis label

figure(fig_no); sgtitle(sgtitle_name)

subplot(2, 4, 1); 
plot(T, DATA1(:, 1), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 1), 'b', 'LineWidth', 0.5);
ylabel(ylabel_name); title('Joint 1'); 

subplot(2, 4, 2); 
plot(T, DATA1(:, 2), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 2), 'b', 'LineWidth', 0.5);
title('Joint 2');

subplot(2, 4, 3); 
plot(T, DATA1(:, 3), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 3), 'b', 'LineWidth', 0.5);
title('Joint 3');

subplot(2, 4, 4); 
plot(T, DATA1(:, 4), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 4), 'b', 'LineWidth', 0.5);
xlabel('Time Elapsed[s]'); title('Joint 4');

subplot(2, 4, 5); 
plot(T, DATA1(:, 5), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 5), 'b', 'LineWidth', 0.5);
ylabel(ylabel_name); xlabel('Time Elapsed[s]'); title('Joint 5');

subplot(2, 4, 6); 
plot(T, DATA1(:, 6), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 6), 'b', 'LineWidth', 0.5) 
xlabel('Time Elapsed[s]'); title('Joint 6');

subplot(2, 4, 7); 
plot(T, DATA1(:, 7), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 7), 'b', 'LineWidth', 0.5); 
xlabel('Time Elapsed[s]'); title('Joint 7');