function [] = test4()
addpath(genpath('./vrep'));

vrep = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
%% 初始化
if(clientID > -1)
    disp('*** Connected to remote API server ***');
    [~, dep_camera1] = vrep.simxGetObjectHandle(clientID, 'kinect_depth1', vrep.simx_opmode_oneshot_wait); % obtain the handle of depth camera
    [~, rgb_camera1] = vrep.simxGetObjectHandle(clientID, 'kinect_rgb1', vrep.simx_opmode_oneshot_wait); % obtain the handle of RGB camera
    [~, dep_camera2] = vrep.simxGetObjectHandle(clientID, 'kinect_depth2', vrep.simx_opmode_oneshot_wait); % obtain the handle of depth camera
    [~, rgb_camera2] = vrep.simxGetObjectHandle(clientID, 'kinect_rgb2', vrep.simx_opmode_oneshot_wait); % obtain the handle of RGB camera
    [~, sensorHandle1] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor1', vrep.simx_opmode_oneshot_wait);
    [~, sensorHandle2] = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor2', vrep.simx_opmode_oneshot_wait);
else
    error('Failed connecting to remote API server'); 
end
t=clock;
currentTime=t(5)*60+t(6);
startTime=t(5)*60+t(6);
Kp = 0.01
Ki = 0.025
Kd = 0.01
pre_err = 0;
d_err = 0;
dd_err = 0;
pre_d_err = 0;
%% 开始仿真
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot); % start simulation
[~, detectionState1, ~, ~, ~] = vrep.simxReadProximitySensor (clientID, sensorHandle1, vrep.simx_opmode_oneshot_wait);
%拾取物体
if detectionState1 == 1
    vrep.simxSetStringSignal(clientID, 'step1', 1, vrep.simx_opmode_oneshot);
    pause(5)
    vrep.simxSetStringSignal(clientID, 'step2', 1, vrep.simx_opmode_oneshot);
    pause(1)
    vrep.simxSetStringSignal(clientID, 'step3', 1, vrep.simx_opmode_oneshot);
    pause(3)
    vrep.simxSetStringSignal(clientID, 'step4', 1, vrep.simx_opmode_oneshot);
    pause(1)
    vrep.simxSetStringSignal(clientID, 'step5', 1, vrep.simx_opmode_oneshot);
    pause(3)
    vrep.simxSetStringSignal(clientID, 'step6', 1, vrep.simx_opmode_oneshot);
    pause(5)
    vrep.simxSetStringSignal(clientID, 'step7', 1, vrep.simx_opmode_oneshot);
    pause(1)
    vrep.simxSetStringSignal(clientID, 'step8', 1, vrep.simx_opmode_oneshot);
    pause(2)
end
%寻标
while (currentTime-startTime < 5000)
    [~, ~, rgb_vision1] = vrep.simxGetVisionSensorImage2(clientID, rgb_camera1, 0, vrep.simx_opmode_oneshot_wait); % obtain the vision of RGB camera
    [~, ~, rgb_vision2] = vrep.simxGetVisionSensorImage2(clientID, rgb_camera2, 0, vrep.simx_opmode_oneshot_wait); % obtain the vision of RGB camera
    [~, detectionState1, ~, ~, ~] = vrep.simxReadProximitySensor (clientID, sensorHandle1, vrep.simx_opmode_oneshot_wait);
    [~, detectionState2, ~, ~, ~] = vrep.simxReadProximitySensor (clientID, sensorHandle2, vrep.simx_opmode_oneshot_wait);
   %避障
   if detectionState1 == 1
        forwBackVel = 0;
        leftRightVel = 1;
        rotVel = 0;
        packedData = vrep.simxPackFloats([forwBackVel, leftRightVel, rotVel]);
        vrep.simxSetStringSignal(clientID, 'control_data', packedData, vrep.simx_opmode_oneshot);
        pause(1);
    elseif detectionState2 == 1
        forwBackVel = 0;
        leftRightVel = -1;
        rotVel = 0;
        packedData = vrep.simxPackFloats([forwBackVel, leftRightVel, rotVel]);
        vrep.simxSetStringSignal(clientID, 'control_data', packedData, vrep.simx_opmode_oneshot);
        pause(1);
     end
    
    %寻找红色色块
    x1 = rgb_vision1(:,:,1);
    y1 = rgb_vision1(:,:,2);
    z1 = rgb_vision1(:,:,3);
    Ir1 = x1*2 - y1 - z1;
    [m1,n1]=size(Ir1);
    Ir1=medfilt2(Ir1,[5,5]);%进行中值滤波;
    [b1,c1]=find(Ir1>100);
    
    x2 = rgb_vision2(:,:,1);
    y2 = rgb_vision2(:,:,2);
    z2 = rgb_vision2(:,:,3);
    Ir2 = x2*2 - y2 - z2;
    [m2,n2]=size(Ir2);
    Ir2=medfilt2(Ir2,[5,5]);%进行中值滤波;
    [b2,c2]=find(Ir2>100);
    %如果前视野中发现红色色块
    if c1 ~= 0
        b_mean=mean(b1);
        c_mean=mean(c1);
        P_x=fix(c_mean);
        %text(P_x,P_y,'.');
        P_errx = P_x - n1 * 2 / 5;
        
        d_err = P_errx - pre_err;
        dd_err =d_err - pre_d_err;
        pre_err = P_errx;
        pre_d_err = d_err;
        
        
        forwBackVel = 1.5;
        leftRightVel = Kp * d_err + Ki * P_errx + Kd * dd_err;
        rotVel = 0;
    %如果后视野中发现红色色块          
    elseif c2 ~= 0
        b_mean=mean(b2);
        c_mean=mean(c2);
        P_x=fix(c_mean);
        P_errx = P_x - n2 * 3/ 5;
        
        d_err = P_errx - pre_err;
        dd_err =d_err - pre_d_err;
        pre_err = P_errx;
        pre_d_err = d_err;
        
        forwBackVel = -1.5;
        leftRightVel = -(Kp * d_err + Ki * P_errx + Kd * dd_err);
        rotVel = 0;
    
    else
        forwBackVel = 0; 
        leftRightVel = 0;
        rotVel = 1;
    end
    %发送数据到vrep
    packedData = vrep.simxPackFloats([forwBackVel, leftRightVel, rotVel]);
    vrep.simxSetStringSignal(clientID, 'control_data', packedData, vrep.simx_opmode_oneshot);
end








