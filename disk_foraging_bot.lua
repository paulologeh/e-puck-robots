-- Paul Ologeh
-- 29th April 2019
-- The University of Sheffield
-- Program to control Epuck robot to collect disks. Program uses camera to scan for green disks and finds nearest disk and moves toward that disk.

-- Epuck principal control script
threadFunction=function()
    SimTime = 1 -- Initialised time
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        
-- Image Processing
    sim.handleVisionSensor(ePuckCam) -- the image processing camera is handled explicitely
    result,t0,t1=sim.readVisionSensor(ePuckCam) -- read image processing camera
    
-- The e-puck robot has Blob Detection filter. The code provided below gets useful information
-- regarding blobs detected, such as amount, size, position, etc.
    
    if (t1) then -- (if Detection is successful) in t1 we should have the blob information if the camera was set-up correctly  
        blobCount=t1[1] -- number of blobs recognised
        dataSizePerBlob=t1[2]
        lowestYofDetection=100
        largest= t1[2+(1-1)*dataSizePerBlob+1] -- initialise the largest blob size to the first blob
        -- Loop through all blobs
        for i=1,blobCount,1 do
            blobSize=t1[2+(i-1)*dataSizePerBlob+1] -- size of blob image
            blobOrientation=t1[2+(i-1)*dataSizePerBlob+2]
            blobPos={t1[2+(i-1)*dataSizePerBlob+3],t1[2+(i-1)*dataSizePerBlob+4]} -- x and y coordinates of the current blob
            blobBoxDimensions={t1[2+(i-1)*dataSizePerBlob+5],t1[2+(i-1)*dataSizePerBlob+6]}
            -- updates variable 'largest' with the x position of the current largest blob
            if (blobSize>=largest) then
                largest = blobSize
                X = blobPos[1]
            end
        end
    end

--  Retrieve proximity sensor readings
        -- Scale robot to simulation
        s=sim.getObjectSizeFactor(bodyElements) -- make sure that if we scale the robot during simulation, other values are scaled too!
        noDetectionDistance=0.05*s
        proxSensDist={noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance,noDetectionDistance}
        for i=1,8,1 do  -- loop through all sensors
            res,dist=sim.readProximitySensor(proxSens[i])
            if (res>0) and (dist<noDetectionDistance) then
                proxSensDist[i]=dist -- store values of current sensor
            end
        end

        kp = 4.48 -- choose proportional gain
        default = 4 -- default speed in rad/s.
       sim.addStatusbarMessage(SimTime)
          if (proxSensDist[3]<0.05) or (proxSensDist[4]<0.05) then -- check if oject is close
            if(proxSensDist[3]<proxSensDist[4]) then  -- obstacle on the left so turn right
                velLeft = 6.24
                velRight = -6.24
            else  -- else obstacle is on the right so turn left
                velLeft = -6.24
                velRight = 6.24
            end
        else          
            if (t1[1]==0) then -- if blob detected
                    SimTime = SimTime + sim.getSimulationTime()  -- start timer
                    -- Search moving clockwise for 3 seconds and switch to anticlockwise search if nothing for 3 seconds
                    if(SimTime<3000) or (SimTime~= null) then 
                        velLeft= default + kp
                        velRight= default - kp/2
                    else
                        velLeft = default -kp/2
                        velRight = default + kp
                    end

            else
                error = X - 0.5  -- calculate error
                proportional = error * kp  -- determine proportional value
                --- Adjust wheels to reduce error & reset timer
                velLeft  = default + proportional  
                velRight = default - proportional
                SimTime = 0
                if(error<0.01) and (error>-0.01)then  -- override default speed of 4 rad/s to max speed when error is very small
                    velLeft = 6.24
                    velRight = 6.24
                end
            end
        end
        sim.setJointTargetVelocity(leftMotor,velLeft)  -- set left velocity
        sim.setJointTargetVelocity(rightMotor,velRight) -- set right velocity
        sim.switchThread() 
    end
end


-- Declare Handles
sim.setThreadSwitchTiming(200)
bodyElements=sim.getObjectHandle('ePuck_bodyElements')
leftMotor=sim.getObjectHandle('ePuck_leftJoint')
rightMotor=sim.getObjectHandle('ePuck_rightJoint')
ePuck=sim.getObjectHandle('ePuck')
ePuckCam=sim.getObjectHandle('ePuck_camera')
ePuckBase=sim.getObjectHandle('ePuck_base')
ledLight=sim.getObjectHandle('ePuck_ledLight')

proxSens={-1,-1,-1,-1,-1,-1,-1,-1}
for i=1,8,1 do
    proxSens[i]=sim.getObjectHandle('ePuck_proxSensor'..i)
end


res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    sim.addStatusbarMessage('Lua runtime error: '..err)
end