function [clientID, vrep, jointHandles, endEffectorHandle, baseFrameHandle] = initializeConnectionAndHandles(jointNames, endEffectorName, baseFrameName)

% 设置远程API的连接
vrep=remApi('remoteApi'); % 远程API类
vrep.simxFinish(-1); % just in case, close all opened connections
% 连接到CoppeliaSim（端口号通常是19997）
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if clientID == -1
    disp('无法连接到CoppeliaSim');
    return;
end
disp('连接成功');

vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking);
pause(0.5); % 等待仿真稳定

% 获取每个关节的句柄
% jointNames = {'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'};
jointHandles = zeros(1, length(jointNames));
for i = 1:length(jointNames)
    [res, jointHandles(i)] = vrep.simxGetObjectHandle(clientID, jointNames{i}, vrep.simx_opmode_blocking);
    if res ~= vrep.simx_return_ok
        error(['无法获取关节句柄：', jointNames{i}]);
    end
end
disp('所有关节句柄获取成功');


% 获取机械臂末端执行器的句柄
[res, endEffectorHandle] = vrep.simxGetObjectHandle(clientID, endEffectorName, vrep.simx_opmode_blocking);
if res ~= vrep.simx_return_ok
    disp('无法获取末端执行器句柄');
    return;
end
disp('末端执行器句柄获取成功');


% 获取基坐标系的句柄
[res, baseFrameHandle] = vrep.simxGetObjectHandle(clientID, baseFrameName, vrep.simx_opmode_blocking);
if res ~= vrep.simx_return_ok
    disp('无法获取基坐标系句柄');
    return;
end
disp('基坐标系句柄获取成功');

end
