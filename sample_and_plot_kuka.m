clear pose_errors_sam t_sam
close all

num = N_steps + 1;
interval = 5;%每隔一定数目抽取一个值
sam = 0;
epsilon = 1;
t = 0: 0.01 : 15;
for j1=1:num
    if(j1>=epsilon)
        sam=sam+1;
        pose_errors_sam(:, sam) = pose_errors_axang(:, j1);
        t_sam(:, sam) = t(j1);
        epsilon=j1+interval;  %抽样间隔
    elseif(j1==num) %确保最后一个数值被抽取
        sam=sam+1;
        pose_errors_sam(:, sam) = pose_errors_axang(:, j1);
        t_sam(:, sam) = t(j1);
    end
end

%绘图------------------------------------------------------------------------%
figure(1);
% 子图1：位置误差
subplot(2,1,1);
hold on;
box on;
% 绘制位置误差曲线
plot(t_sam, pose_errors_sam(1,:), '-b', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_x$'); % 蓝色实线
plot(t_sam, pose_errors_sam(2,:), '--r', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_y$'); % 红色虚线
plot(t_sam, pose_errors_sam(3,:), ':g', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_z$'); % 绿色点线
% 设置图例
legend('Interpreter', 'latex', 'FontSize', 14, 'FontName', 'Times New Roman', 'NumColumns', 3, 'Location', 'northeast');
% 设置Y轴标签
ylabel('$\mathbf{e}_\mathrm{p}$ (m)', 'Interpreter', 'latex', 'FontSize', 18, 'FontName', 'Times New Roman', 'rotation', 0);
% 自动调整Y轴范围
ylim padded;
% 设置子图格式
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14, 'Position', [0.1 0.575 0.85 0.375], 'LineWidth', 1.5);

% 子图2：轴角误差
subplot(2,1,2);
hold on;
box on;
% 绘制轴角误差曲线
plot(t_sam, pose_errors_sam(7,:), '-b', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_\alpha$'); % 紫色实线
plot(t_sam, pose_errors_sam(4,:), '-.r', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_{\gamma_{z}}$'); % 黄色点划线
plot(t_sam, pose_errors_sam(5,:), '--g', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_{\gamma_{x}}$'); % 青色虚线
plot(t_sam, pose_errors_sam(6,:), ':m', 'LineWidth', 2.0, 'DisplayName', '$\mathrm{e}_{\gamma_{y}}$'); % 黑色点线
% 设置图例  
legend('Interpreter', 'latex', 'FontSize', 14, 'FontName', 'Times New Roman', 'NumColumns', 2, 'Location', 'northeast');
% 设置X轴和Y轴标签
xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 16, 'FontName', 'Times New Roman');
ylabel('$\mathbf{e}_\mathrm{ang}$ (rad)', 'Interpreter', 'latex', 'FontSize', 18, 'FontName', 'Times New Roman', 'rotation', 0);
% 自动调整Y轴范围
ylim padded;
% 设置子图格式
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14, 'Position', [0.1 0.1 0.85 0.375], 'LineWidth', 1.5);
