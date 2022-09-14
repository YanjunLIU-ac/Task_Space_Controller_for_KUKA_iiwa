%% Plot two curves per cartesian-axis for comparison in a figure
% @author: yanjun liu
% @date: Aug, 2022

function plotCartFiguresComp(fig_no, T, DATA1, DATA2, sgtitle_name, ylabel_name)
% @param[in] fig_no: number of figure handle
% @param[in] sgtitle_name: total name of figure
% @param[in] ylabel_name: name of y-axis label

figure(fig_no); sgtitle(sgtitle_name)
subplot(1, 3, 1); 
plot(T, DATA1(:, 1), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 1), 'b', 'LineWidth', 0.5);
ylabel(ylabel_name); title('X-axis');

subplot(1, 3, 2); 
plot(T, DATA1(:, 2), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 2), 'b', 'LineWidth', 0.5);
title('Y-axis');

subplot(1, 3, 3); 
plot(T, DATA1(:, 3), 'r', 'LineWidth', 0.5); hold on;
plot(T, DATA2(:, 3), 'b', 'LineWidth', 0.5); 
title('Z-axis');