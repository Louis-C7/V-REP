-- IF/FK 模式转换
enableIk=function(enable)  
    if enable then
        sim.setObjectMatrix(ik_target,-1,sim.getObjectMatrix(ik_tip,-1))
        for i=1,#joint_handles,1 do
            sim.setJointMode(joint_handles[i],sim.jointmode_ik,1)
        end

        sim.setExplicitHandling(ik_group_handle,0)
    else
        sim.setExplicitHandling(ik_group_handle,1)
        for i=1,#joint_handles,1 do
            sim.setJointMode(joint_handles[i],sim.jointmode_force,0)
        end
    end
end
-- 等待小车到达指定位姿
waitToReachVehicleTargetPositionAndOrientation=function()
    repeat
        sim.switchThread() 
        p1=sim.getObjectPosition(youbot_target,-1)
        p2=sim.getObjectPosition(youbot_handle,-1)
        p={p2[1]-p1[1],p2[2]-p1[2]}
        pError=math.sqrt(p[1]*p[1]+p[2]*p[2])  -- 位置差
        oError=math.abs(sim.getObjectOrientation(youbot_handle,youbot_target)[3])  -- 方向差
    until (pError<0.001)and(oError<0.1*math.pi/180) 
end
-- 获取球与小车的位姿差
getSphereAdjustedMatrixAndFacingAngle=function(sphere)
    pos=sim.getObjectPosition(sphere,-1)  -- 获取球的位置
    m=sim.getObjectMatrix(sphere,-1)   -- 获取球相对绝对坐标系的位姿矩阵
    faceX={-m[1],-m[5],-m[9]}  -- 其中第1、5、9个元素代表球坐标系在绝对坐标系中的表示，如果要面向球的x轴的负方向，就需要加符号
    angle=math.atan2(faceX[2],faceX[1])  -- 用反正切求角度
    m=sim.buildMatrix(pos,{0,0,angle})  -- 创建新的位姿矩阵
    return m, angle-math.pi/2
end
--打磨球体  
function polish(flag,sphere_ref,t)     
    --flag==0从上到下；flag==1从下到上
    -- sphere_ref：球的参考坐标系
    -- t：沿z轴旋转的打磨角度
    sim.setThreadAutomaticSwitch(false)  -- 不要打断程序
    sim.setObjectOrientation(sphere_ref,-1,{0,0,t})
    sphere_centre=sim.getObjectPosition(sphere_ref,-1) --球体的中心的坐标（centre_x,centre_y,centre_z）
    sphere_r=0.025                                     --球体的半径
    centre_x=sphere_centre[1]
    centre_y=sphere_centre[2]
    centre_z=sphere_centre[3]

    if(flag==0) then
        alp=0*math.pi/180
    else
        alp=math.pi
    end                           
    if(flag==0)then
        while(alp<=math.pi) do   --从上到下
            --(contact_goal_x,contact_goal_y,contact_goal_z) 末端执行器在球体表面的目的接触位置（球的坐标系下）                      
            contact_goal_z=centre_z + sphere_r*math.cos(alp)                   
            R=math.sqrt(math.abs(sphere_r^2-(sphere_r*math.cos(alp))^2))       --R=sqrt(|r^2-(r*cos(alp))^2|)
            contact_goal_y=centre_y                        
            contact_goal_x=centre_x+R                          
            T=sim.getObjectMatrix(sphere_ref,-1)             --获得球体参考坐标的位姿矩阵T（相对于世界坐标系）
            x=T[1]*contact_goal_x+T[2]*contact_goal_y+T[3]*contact_goal_z      -- 将目标点坐标（相对于球心坐标系）与旋转算子相乘，获得目标点在世界坐标系下的位置                  
            y=T[5]*contact_goal_x+T[6]*contact_goal_y+T[7]*contact_goal_z
            z=T[9]*contact_goal_x+T[10]*contact_goal_y+T[11]*contact_goal_z                   

            T=sim.rotateAroundAxis(T,{1,0,0},{x,y,z},math.pi)                  --矩阵绕x轴在坐标(x,y,z))处旋转pi
            T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)                        --矩阵绕z轴在坐标(x,y,z)处旋转t
            theta=math.asin(R/sphere_r)                                        --目标接触点连接球心后与z轴的夹角

            if(alp>math.pi/2) then   --分为上下半球，调整T矩阵中的姿态为目标接触点相对于圆心的姿态矩阵
                T=sim.rotateAroundAxis(T,{0,1,0},{0,0,0},-theta+math.pi)
                T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)
            else
                T=sim.rotateAroundAxis(T,{0,1,0},{0,0,0},theta)
                T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)
            end
            
            quat=sim.getQuaternionFromMatrix(T)      --在目标点的姿态确定，四元数（旋转轴，旋转角）
            print('quat',quat)
            pos={x,y,z}
            print('pos',pos)
            res=sim.rmlMoveToPosition(ik_target,-1,-1,nil,{0,0,0,0},{1,1,1,15*math.pi/180},ikMaxAccel,ikMaxJerk,pos,quat,nil)
            print('finish 1')                         --已经打磨一个半圆
            alp=alp+0.1
            print('alp',alp)
        end
    else
        while(alp>0) do  -- 从下到上
            --(contact_goal_x,contact_goal_y,contact_goal_z) 末端执行器在球体表面的目的接触位置 （球的坐标系下）                       
            contact_goal_z=centre_z + sphere_r*math.cos(alp)                   
            R=math.sqrt(math.abs(sphere_r^2-(sphere_r*math.cos(alp))^2))          --R=sqrt(|r^2-(r*cos(alp))^2|)
            contact_goal_y=centre_y+R*math.sin(0)            
            contact_goal_x=centre_x+R*math.cos(0)                                                     
            T=sim.getObjectMatrix(sphere_ref,-1)                                  --获得球体参考坐标的位姿矩阵T（相对于世界坐标系）
            --print(T)
            x=T[1]*contact_goal_x+T[2]*contact_goal_y+T[3]*contact_goal_z         -- 将目标点坐标（相对于球心坐标系）与旋转算子相乘，获得目标点在世界坐标系下的位置                  
            y=T[5]*contact_goal_x+T[6]*contact_goal_y+T[7]*contact_goal_z
            z=T[9]*contact_goal_x+T[10]*contact_goal_y+T[11]*contact_goal_z                   

            --T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)
            T=sim.rotateAroundAxis(T,{1,0,0},{x,y,z},math.pi)                     --矩阵绕x轴在坐标(x,y,z))处旋转pi
            T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)                           --矩阵绕z轴在坐标(x,y,z)处旋转t
            theta=math.asin(R/sphere_r)
            
            if(alp>math.pi/2) then  -- 分为上下半球，调整T矩阵中的姿态为目标接触点相对于圆心的姿态矩阵
                T=sim.rotateAroundAxis(T,{0,1,0},{0,0,0},-theta+math.pi)
                T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)
            else
                T=sim.rotateAroundAxis(T,{0,1,0},{0,0,0},theta)
                T=sim.rotateAroundAxis(T,{0,0,1},{x,y,z},t)
            end         
            quat=sim.getQuaternionFromMatrix(T)                      --在目标点的姿态确定，四元数（旋转轴，旋转角）
            print('quat',quat)
            pos={x,y,z}
            print('pos',pos)
            res=sim.rmlMoveToPosition(ik_target,-1,-1,nil,{0,0,0,0},{1,1,1,15*math.pi/180},ikMaxAccel,ikMaxJerk,pos,quat,nil)      
            print('finish 1')                                      --已经打磨一个半圆
            alp=alp-0.1
        end
    end
        print('Scraping finish.')
        sim.setThreadAutomaticSwitch(true)
end
function sysCall_threadmain()

    -- 初始化
    joint_handles={-1,-1,-1,-1,-1,-1,-1}
    for i=1,#joint_handles,1 do
        joint_handles[i]=sim.getObjectHandle('redundantRob_joint'..i)
    end
    ik_group_handle=sim.getIkGroupHandle('redundantRobot_damped')
    sim.setExplicitHandling(ik_group_handle,1)
    ik_tip=sim.getObjectHandle('redundantRob_tip')
    ik_target=sim.getObjectHandle('redundantRob_target')
    youbot_handle=sim.getObjectHandle('youBot_vehicleReference')
    youbot_target=sim.getObjectHandle('youBot_vehicleTargetPosition')
    model_base=sim.getObjectAssociatedWithScript(sim.handle_self)
    model_name=sim.getObjectName(model_base)
    sphere_ref=sim.getObjectHandle('Sphere_Ref')
    -- Set-up some of the RML vectors:
    vel=180
    accel=40
    jerk=80
    currentVel={0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0,0}
    ikMaxVel={0.5,0.5,0.5,1.72}
    ikMaxAccel={5,5,5,3.14}
    ikMaxJerk={500,500,500,500}
    pickConfig={0*math.pi/180,-30*math.pi/180,0*math.pi/180,170*math.pi/180,0*math.pi/180,0*math.pi/180,0}
    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
    -- 机械臂移动到初始位姿
        enableIk(false)
        sim.rmlMoveToJointPositions(joint_handles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,pickConfig,targetVel)
    end
    -- 切换回IK模式
    if sim.getSimulationState()~=sim.simulation_advancing_abouttostop then
        sim.setObjectMatrix(ik_target,-1,sim.getObjectMatrix(ik_tip,-1))
        for i=1,#joint_handles,1 do
            sim.setJointMode(joint_handles[i],sim.jointmode_ik,1)
        end
        sim.setExplicitHandling(ik_group_handle,0)
    end
    dist1=0.1    -- 小车离球的距离
    m,angle=getSphereAdjustedMatrixAndFacingAngle(sphere_ref)
    sim.setObjectPosition(youbot_target,-1,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
    sim.setObjectOrientation(youbot_target,-1,{0,0,angle})
    waitToReachVehicleTargetPositionAndOrientation()  --等待小车到达指定位姿
    t=0
    oritntation=sim.getObjectOrientation(sphere_ref,-1)
    r=math.sqrt((m[4]-m[1]*dist1)^2+(m[8]-m[5]*dist1)^2)  --小车绕球旋转时的半径

    while(t<2*math.pi) do
        polish(0,sphere_ref,t)    -- 从上到下打磨半圈
        -- 小车绕球旋转t
        t=t+0.2
        sim.setObjectPosition(youbot_target,-1,{r*math.cos(t),r*math.sin(t),0})
        sim.setObjectOrientation(youbot_target,-1,{0,0,t+math.pi/2})   
        --waitToReachVehicleTargetPositionAndOrientation()
        polish(1,sphere_ref,t)    -- 从下到上打磨半圈
        -- 小车绕球旋转t
        t=t+0.2
        sim.setObjectPosition(youbot_target,-1,{r*math.cos(t),r*math.sin(t),0})
        sim.setObjectOrientation(youbot_target,-1,{0,0,t+math.pi/2})  
    end
    -- 打磨结束，退后然后结束仿真
    dist1=0.5
    sim.setObjectPosition(youbot_target,-1,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
    waitToReachVehicleTargetPositionAndOrientation()
    sim.stopSimulation()
end