function sysCall_init()
         
    vehicleReference=sim.getObjectHandle('youBot_vehicleReference')      -- 小车的参考坐标系
    vehicleTarget=sim.getObjectHandle('youBot_vehicleTargetPosition')    -- 小车目标点
    sim.setObjectPosition(vehicleTarget,sim.handle_parent,{0,0,0})
    sim.setObjectOrientation(vehicleTarget,sim.handle_parent,{0,0,0})
    sim.setObjectParent(vehicleTarget,-1,true)
    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=sim.getObjectHandle('rollingJoint_fl')
    wheelJoints[2]=sim.getObjectHandle('rollingJoint_rl')
    wheelJoints[3]=sim.getObjectHandle('rollingJoint_rr')
    wheelJoints[4]=sim.getObjectHandle('rollingJoint_fr')
    previousForwBackVel=0   -- 上一个循环的前后速度
    previousLeftRightVel=0  -- 上一个循环的左右速度
    previousRotVel=0        -- 上一个循环的旋转速度
    
end

function sysCall_cleanup() 
 
end 

function sysCall_actuation() 
    relP=sim.getObjectPosition(vehicleTarget,vehicleReference)     -- 位置差
    relE=sim.getObjectOrientation(vehicleTarget,vehicleReference)  -- 角度差
    pParam=20      -- P参数
    maxV=2         -- 最大线速度
    pParamRot=10   
    maxVRot=3      -- 最大角速度
    accelF=0.035   -- 加速度参数
    
    
    forwBackVel=relP[2]*pParam    
    leftRightVel=relP[1]*pParam
    v=math.sqrt(forwBackVel*forwBackVel+leftRightVel*leftRightVel)  -- 线速度
    --调整速度
    if v>maxV then
        forwBackVel=forwBackVel*maxV/v
        leftRightVel=leftRightVel*maxV/v
    end
    rotVel=-relE[3]*pParamRot  -- 角速度
    if (math.abs(rotVel)>maxVRot) then
        rotVel=maxVRot*rotVel/math.abs(rotVel)
    end
    -- 速度的变化量
    df=forwBackVel-previousForwBackVel
    ds=leftRightVel-previousLeftRightVel
    dr=rotVel-previousRotVel
    -- 调整加速度，不要超过最大值
    if (math.abs(df)>maxV*accelF) then
        df=math.abs(df)*(maxV*accelF)/df
    end
    
    if (math.abs(ds)>maxV*accelF) then
        ds=math.abs(ds)*(maxV*accelF)/ds
    end
    
    if (math.abs(dr)>maxVRot*accelF) then
        dr=math.abs(dr)*(maxVRot*accelF)/dr
    end
    

    forwBackVel=previousForwBackVel+df
    leftRightVel=previousLeftRightVel+ds
    rotVel=previousRotVel+dr
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
    -- 保存速度
    previousForwBackVel=forwBackVel
    previousLeftRightVel=leftRightVel
    previousRotVel=rotVel
end 

