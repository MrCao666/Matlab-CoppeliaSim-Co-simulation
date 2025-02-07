function J_init = Jacobian_Estimator(vrep, clientID, jointHandles, based, handleEndEffector, currentAngle)

% 定义小幅度的关节变化量
deltaTheta = 0.05; % 每个关节的角度变化量
J_init = zeros(8, length(jointHandles)); % 初始化雅可比矩阵的存储

% 获取末端执行器初始位姿
[~, initialPos] = vrep.simxGetObjectPosition(clientID, handleEndEffector, based, vrep.simx_opmode_blocking);
[~, initialOr] = vrep.simxGetObjectOrientation(clientID, handleEndEffector, based, vrep.simx_opmode_blocking);
initialOr = reshape(eul2rotm(initialOr, 'XYZ'), [], 1);

for i = 1:7
    
    % 施加小幅度的角度变化
    newAngle = currentAngle(i) + deltaTheta;
    vrep.simxSetJointTargetPosition(clientID, jointHandles(i), newAngle, vrep.simx_opmode_blocking);

    % 计算末端执行器的新位姿
    pause(0.1); % 等待机械臂移动一小段时间
    [~, newPos] = vrep.simxGetObjectPosition(clientID, handleEndEffector, based, vrep.simx_opmode_blocking);
    [~, newOr] = vrep.simxGetObjectOrientation(clientID, handleEndEffector, based, vrep.simx_opmode_blocking);
    newOr = reshape(eul2rotm(newOr, 'XYZ'), [], 1);

    % 计算位姿变化
    deltaPos = newPos' - initialPos';
    deltaOr = newOr(5:9) - initialOr(5:9);
    deltaS = [deltaPos; deltaOr]; % 位姿变化量

    % 估算雅可比矩阵的第i列
    J_init(:, i) = deltaS / deltaTheta; % 用公式(4)进行计算

    % 关节复位
    vrep.simxSetJointTargetPosition(clientID, jointHandles(i), currentAngle(i), vrep.simx_opmode_blocking);
end

% 打印估算出的初始雅可比矩阵
% disp('初始雅可比矩阵：');
% disp(J_init);

% % 关节复位
% for i = 1:length(jointHandles)
%     res = vrep.simxSetJointTargetPosition(clientID, jointHandles(i), currentAngle(i), vrep.simx_opmode_blocking);
%     %     disp(['Joint ', jointNames{i}, ' 返回值: ', num2str(res)]); % 打印返回值
%     if res ~= vrep.simx_return_ok && res ~= vrep.simx_return_novalue_flag
%         error(['无法发送目标角度到关节，返回值为：', num2str(res)]);
%     end
% end
% % pause(0.1);

end
