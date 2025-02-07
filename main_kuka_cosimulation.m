
clear;
clc;
close all;  

% 定义关节名称和末端执行器名称
jointNames = {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'};
endEffectorName = 'end';
baseFrameName = 'based';

[clientID, vrep, jointHandles, endEffectorHandle, baseFrameHandle] = initializeConnectionAndHandles(jointNames, endEffectorName, baseFrameName);

% 时间步长和预测/控制域
T = 15; % 总时间 (s)
sigma = 0.01;  % 时间步长
N_p = 5;       % 预测域
N_c = 5;       % 控制域
n = 7;         % 机械臂关节数
m = 8;         % 末端位姿维度
N_steps = T / sigma; % 总采样点数

% 初始化记录变量
joint_angles = zeros(n, N_steps); % 关节角度
joint_velocities = zeros(n, N_steps); % 关节速度
joint_accelerations = zeros(n, N_steps); % 关节加速度
actual_poses = zeros(m, N_steps); % 实际末端位姿（旋转向量姿态）
actual_poses_axang = zeros(7, N_steps); % 实际末端位姿（轴角姿态）
desired_poses_axang = zeros(7, N_steps); % 期望末端位姿
pose_errors_axang = zeros(7, N_steps); % 末端位姿误差
hat_J = zeros(m, n, N_steps); 

% 初始化低通滤波器输出
s_f = zeros(8, N_steps); % 滤波后位姿
v_f = zeros(8, N_steps); % 滤波后速度
ds_f = zeros(8, N_steps); % 滤波后位姿的一阶导数
dv_f = zeros(8, N_steps); % 滤波后速度的一阶导数

%预备量--------------------------------------------------------------------%
% 定义关节角度的上下限（弧度）
theta_max = [2.96; 2.09; 2.96; 2.09; 2.96; 2.09; 3.05]; % 最大角度
theta_min = [-2.96; -2.09; -2.96; -2.09; -2.96; -2.09; -3.05]; % 最小角度
% 定义关节速度的上下限（弧度/秒）
theta_dot_max = [1.71; 1.71; 1.75; 2.27; 2.44; 3.14; 3.14]; % 最大速度
theta_dot_min = -theta_dot_max; % 最小速度（对称）
% 定义关节加速度的上下限（弧度/秒²）
theta_ddot_max = [20; 20; 20; 20; 20; 20; 20]; % 最大加速度
theta_ddot_min = -theta_ddot_max; % 最小加速度（对称）

% 构建关节角度限制矩阵
Theta_max = repmat(theta_max', N_p, 1); % 将 theta_max 重复 N_p 次
Theta_min = repmat(theta_min', N_p, 1); % 将 theta_min 重复 N_p 次
% 构建关节速度限制矩阵
Theta_dot_max = repmat(theta_dot_max', N_c, 1); % 将 theta_dot_max 重复 N_c 次
Theta_dot_min = repmat(theta_dot_min', N_c, 1); % 将 theta_dot_min 重复 N_c 次
% 构建关节加速度限制矩阵
Theta_ddot_max = repmat(theta_ddot_max', N_c, 1); % 将 theta_ddot_max 重复 N_c 次
Theta_ddot_min = repmat(theta_ddot_min', N_c, 1); % 将 theta_ddot_min 重复 N_c 次

% 权重矩阵
Q1_prime = 500000*eye(m);  % 位姿误差权重
Q2_prime = 1*eye(n);  % 关节速度权重
Q3_prime = 20*eye(n);  % 关节加速度权重
Q1 = kron(eye(N_p), Q1_prime);
Q2 = kron(eye(N_c), Q2_prime);
Q3 = kron(eye(N_c), Q3_prime);

% 构建矩阵 C
C = zeros(N_p, N_c); % 初始化 C 矩阵
for i = 1:N_p
    for j = 1:N_c
        if i >= j
            C(i, j) = (i - j + 1) * sigma;
        end
    end
end

% 构建矩阵 tilde(I)
tilde_I = tril(ones(N_c)); % 下三角矩阵，非零元素为 1
% 构建单位矩阵 I
I_Nc = eye(N_c); % 控制步长内的单位矩阵
% 构建矩阵 E'
E_prime = [C; -C; tilde_I; -tilde_I; I_Nc; -I_Nc];
% 构建矩阵 E
E = kron(eye(n), E_prime); % 按关节维度扩展 E'

% --- 构建矩阵 (I ⊗ tilde(I)) ---
I_kron_tildeI = kron(eye(n), tilde_I); % Kronecker 积扩展到控制域和关节数

% 初始化初始状态
joint_angles(:, 1) = deg2rad([0; -30; 0; -110; 0; 85; -80]); % 初始关节角度
joint_velocities(:, 1) = zeros(n, 1); % 初始关节速度
joint_accelerations(:, 1) = zeros(n, 1); % 初始关节加速度

% 设置初始关节构型
for i = 1:length(jointHandles)
    res = vrep.simxSetJointTargetPosition(clientID, jointHandles(i), joint_angles(i, 1), vrep.simx_opmode_blocking);
    %     disp(['Joint ', jointNames{i}, ' 返回值: ', num2str(res)]); % 打印返回值
    if res ~= vrep.simx_return_ok && res ~= vrep.simx_return_novalue_flag
        error(['无法发送目标角度到关节：', jointNames{i}, '，返回值为：', num2str(res)]);
    end
end
% pause(0.1);

% 获取末端执行器位置和旋转矩阵，使用阻塞模式
[resPos, position] = vrep.simxGetObjectPosition(clientID, endEffectorHandle, baseFrameHandle, vrep.simx_opmode_blocking);
[resOri, ori_eul] = vrep.simxGetObjectOrientation(clientID, endEffectorHandle, baseFrameHandle, vrep.simx_opmode_blocking);
if resPos == vrep.simx_return_ok && resOri == vrep.simx_return_ok
    rotm_api = eul2rotm(double(ori_eul),'XYZ');
    actual_poses(:, 1) = [position';rotm_api(2:3,2);rotm_api(1:3,3)];

    axang = rotm2axang(rotm_api);
    actual_poses_axang(:,1) = [position'; axang'];
else
    disp('无法获取末端位置或姿态');
end

% 初始滤波器输出赋值
s_f(:, 1) = actual_poses(:, 1); % 初始滤波后的位姿

% 初始化雅可比矩阵
hat_J(:, :, 1) = Jacobian_Estimator(vrep, clientID, jointHandles, baseFrameHandle, endEffectorHandle, joint_angles(:, 1));

% 初始化数据流
vrep.simxGetObjectPosition(clientID, endEffectorHandle, baseFrameHandle, vrep.simx_opmode_streaming);
vrep.simxGetObjectOrientation(clientID, endEffectorHandle, baseFrameHandle, vrep.simx_opmode_streaming);
pause(0.1); % 等待数据流初始化

% --- 计算初始位姿误差 ---
% 获取期望位姿
[~, desired_poses_axang(:, 1)] = path_reference_kuka_angvec(0); % 获取期望位姿
% 计算误差
pose_errors_axang(:, 1) = actual_poses_axang(:, 1) - desired_poses_axang(:, 1); % 姿态误差

%模型预测控制------------------------------------------------------------------------------
for k = 1:N_steps

    t = (k - 1) * sigma; % 当前时间

    J = hat_J(:, :, k);

    J_kron_C = kron(J, C);

    % --- 计算 M 矩阵 ---
    M = 2 * (J_kron_C' * Q1 * J_kron_C + I_kron_tildeI' * Q2 * I_kron_tildeI + Q3);

    % 构建矩阵 H
    H = repmat(actual_poses(:, k)', N_p, 1);
    % 构建矩阵 S
    S = zeros(N_p, m);
    for i = 1:N_p
        S(i, :) = path_reference_kuka_angvec(t + i * sigma)';
    end
    % 构建矩阵 B
    if k == 1
        B = zeros(N_p, n); % 初始化矩阵
        for i = 1:N_p
            B(i, :) = i * sigma * joint_velocities(:, k)';
        end
    else
        B = zeros(N_p, n); % 初始化矩阵
        for i = 1:N_p
            B(i, :) = i * sigma * joint_velocities(:, k-1)';
        end
    end
    % 构建矩阵 A
    if k == 1
        A = repmat(joint_velocities(:, k)', N_c, 1);
    else
        A = repmat(joint_velocities(:, k-1)', N_c, 1);
    end
    % 计算 vec(H)
    vec_H = reshape(H, [], 1);
    % 计算 vec(B * J')
    vec_B_JT = reshape(B * J', [], 1);
    % 计算 vec(S)
    vec_S = reshape(S, [], 1);
    % 计算第二部分
    I_kron_tildeI = kron(eye(n), tilde_I); % Kronecker 积扩展到控制域
    vec_A = reshape(A, [], 1); % 向量化 A
    m_bf = 2 * J_kron_C' * Q1 * (vec_H + vec_B_JT - vec_S) + 2 * I_kron_tildeI' * Q2 * vec_A;

    % --- 构建矩阵 D ---
    % 当前关节角度扩展到预测域
    D = repmat(joint_angles(:, k)', N_p, 1);
    % --- 构建矩阵 F ---
    F = [
        Theta_max - D - B; % 上限约束
        -Theta_min + D + B; % 下限约束
        Theta_dot_max - A; % 速度上限
        -Theta_dot_min + A; % 速度下限
        sigma * Theta_ddot_max; % 加速度上限
        -sigma * Theta_ddot_min; % 加速度下限
        ];
    % --- 构建向量 c ---
    c = reshape(F, [], 1); % 将矩阵 F 向量化

    % 初始化 ZN 求解器必要参数
    lambda = 1; % 收敛速率参数
    dt = 0.01; % 时间步长
    max_iter = 20; % 最大迭代次数
    tol = 1e-8; % 收敛容差

    % --- 初始化 ZN 求解变量 ---
    v = zeros(n * N_c, 1); % 初始控制输入
    rho = zeros(size(c)); % 初始拉格朗日乘子
    % 初始化 \(\boldsymbol{x} = [\boldsymbol{\upsilon}; \boldsymbol{\varrho}]\)
    x = [v; rho]; % 联合向量
    % 构建矩阵 K
    K = [M, E'; -E, eye(size(E, 1))];
    % 计算 \(\mathbf{w}\) 和 \(\mathbf{d}\)
    w = c - E * v; % 当前约束差值
    delta = 1e-8; % 防止除零的小偏移量
    d = sqrt((w.^2) + (rho.^2) + delta); % 计算 \(\mathbf{d}\)
    % 构建向量 y
    y = [m_bf; c - d];
    % 计算 Z1 和 Z2
    Z1 = diag(w ./ d); % 计算对角矩阵 \(\mathrm{Z}_1\)
    Z2 = diag(rho ./ d); % 计算对角矩阵 \(\mathrm{Z}_2\)
    % 构建矩阵 W
    W = [M, E'; -E + Z1 * E, eye(size(E, 1)) - Z2];
    
    % --- ZN 求解 ---
    for iter = 1:max_iter
        % 计算非线性方程的误差项
        error_term = K * x + y;

        % 更新 \(\dot{\boldsymbol{x}}\) (时间变化率)
        x = x - W \ (lambda * error_term);

        % 更新 \(\boldsymbol{\upsilon}\) 和 \(\boldsymbol{\varrho}\)
        v = x(1:n * N_c); % 控制变量
        rho = x(n * N_c + 1:end); % 拉格朗日乘子

        % 重新计算 \(\mathbf{w}\), \(\mathbf{d}\), \(\mathrm{W}\), 和 \(\boldsymbol{y}\)
        w = c - E * v;
        d = sqrt((w.^2) + (rho.^2) + delta);
        y = [m_bf; c - d];
        Z1 = diag(w ./ d);
        Z2 = diag(rho ./ d);
        W = [M, E'; -E + Z1 * E, eye(size(E, 1)) - Z2];

        % 检查误差收敛
        if norm(error_term, 2) < tol
%             disp(['Converged in ', num2str(iter), ' iterations']);
            break;
        end
    end

    %更新到下一步--------------------------------------------------------------%
    % 从解 \(\boldsymbol{x}\) 中提取控制增量 \(\boldsymbol{\mu}(\kappa)\)
    v = x(1:n * N_c); % 提取控制变量 \(\boldsymbol{\upsilon}\)
    U = reshape(v, [N_c, n]); % 转为控制域矩阵 \(\mathrm{U}\)
    mu = U(1, :)'; % 当前时刻的关节速度增量 \(\boldsymbol{\mu}(\kappa)\)

    % 更新关节速度
    if k == 1
        % 初始时刻的速度更新
        joint_velocities(:, k) = mu; % \(\dot{\boldsymbol{\vartheta}}(0) = \boldsymbol{\mu}(0)\)
    else
        % 非初始时刻的速度更新
        joint_velocities(:, k) = joint_velocities(:, k-1) + mu; % \(\dot{\boldsymbol{\vartheta}}(\kappa) = \boldsymbol{\mu}(\kappa) + \dot{\boldsymbol{\vartheta}}(\kappa-1)\)
    end

    % 计算关节加速度
    joint_accelerations(:, k) = mu / sigma; % \(\ddot{\boldsymbol{\vartheta}}(\kappa) = \boldsymbol{\mu}(\kappa) / \sigma\)

    % 更新关节角度
    joint_angles(:, k + 1) = joint_angles(:, k) + joint_velocities(:, k) * sigma; % \(\boldsymbol{\vartheta}(\kappa+1) = \boldsymbol{\vartheta}(\kappa) + \dot{\boldsymbol{\vartheta}}(\kappa) \cdot \sigma\)
    
    %Jacobian矩阵估计器--------------------------------------------------------%
    tau1 = 1/(200*pi);
    tau2 = 1/(200*pi);
    s_f(:, k + 1) = (1 - sigma/(tau1+sigma)) * s_f(:,k) + sigma/(tau1+sigma) * actual_poses(:, k);
    ds_f(:, k) = (1/tau1) * (actual_poses(:, k) - s_f(:, k + 1));
    v_f(:,k+1) = (1 - sigma/(tau2+sigma)) * v_f(:, k) + sigma/(tau2+sigma) * ds_f(:, k);
    dv_f(:, k) = (1/tau2) * (ds_f(:, k) - v_f(:,k+1));

    % --- 当前雅可比矩阵估计 ---
    theta_dot = joint_velocities(:, k); % 当前关节速度
    theta_ddot = joint_accelerations(:, k); % 当前关节加速度

    % --- 计算雅可比矩阵的时间变化率 ---
    % 初始化 DZN-JMA 调节参数
    eta = 10; % DZN-JMA 调节参数
    theta_dot_inver = (theta_dot' * theta_dot + 0.5) \ theta_dot';% pseudo-inverse
    J_dot = (dv_f(:, k) - J * theta_ddot + eta * (ds_f(:, k) - J * theta_dot)) * theta_dot_inver;

    % --- 离散时间更新雅可比矩阵 ---
    hat_J(:, :, k + 1) = J + sigma * J_dot; % \(\hat{\mathrm{J}}(\kappa+1) = \hat{\mathrm{J}}(\kappa) + \sigma \dot{\hat{\mathrm{J}}}(\kappa)\)

    % --- 将更新后的关节角度发送至 CoppeliaSim ---
    for i = 1:length(jointHandles)
        res = vrep.simxSetJointTargetPosition(clientID, jointHandles(i), joint_angles(i, k + 1), vrep.simx_opmode_oneshot);
        %     disp(['Joint ', jointNames{i}, ' 返回值: ', num2str(res)]); % 打印返回值
        if res ~= vrep.simx_return_ok && res ~= vrep.simx_return_novalue_flag
            error(['无法发送目标角度到关节：', jointNames{i}, '，返回值为：', num2str(res)]);
        end
    end
    % --- 等待机械臂完成运动 ---
    pause(0.03);

    % 获取末端执行器位置和旋转矩阵，使用阻塞模式
    [resPos, position] = vrep.simxGetObjectPosition(clientID, endEffectorHandle, baseFrameHandle, vrep.simx_opmode_buffer);
    [resOri, ori_eul] = vrep.simxGetObjectOrientation(clientID, endEffectorHandle, baseFrameHandle, vrep.simx_opmode_buffer);
    if resPos == vrep.simx_return_ok && resOri == vrep.simx_return_ok
        rotm_api = eul2rotm(double(ori_eul),'XYZ');
        actual_poses(:, k+1) = [position'; rotm_api(2:3,2); rotm_api(1:3,3)];

        axang = rotm2axang(rotm_api);
        actual_poses_axang(:,k+1) = [position'; axang'];
    else
        disp('无法获取末端位置或姿态');
    end

    % --- 计算位姿误差 ---
    % 获取期望位姿
    [~, desired_poses_axang(:, k+1)] = path_reference_kuka_angvec(t + sigma); % 获取期望位姿

    % 计算误差
    pose_errors_axang(:, k+1) = actual_poses_axang(:, k+1) - desired_poses_axang(:, k+1); % 姿态误差

    % --- 打印当前采样周期和时刻 ---
    fprintf('采样周期: %d\n', k);
end

% 暂停仿真
res = vrep.simxPauseSimulation(clientID, vrep.simx_opmode_blocking);
if res == vrep.simx_return_ok
    disp('仿真已暂停');
else
    error('无法暂停仿真');
end

% 断开连接
vrep.simxFinish(clientID);
vrep.delete();
disp('与 CoppeliaSim 的连接已断开');

%绘图------------------------------------------------------------------------%
sample_and_plot_kuka
