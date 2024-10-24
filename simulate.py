import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#p.loadSDF("box.sdf")
robotId = p.loadURDF("body.urdf")

duration = 10000
halfDuration = int(duration/2)
ps.Prepare_To_Simulate(robotId)
x_fast = np.linspace(0, 10 * np.pi, halfDuration)
x_slow = np.linspace(0, 10 * np.pi, halfDuration) 

x = np.concatenate((x_fast, x_slow))
# x = np.linspace(0, 10 * np.pi, duration)
y =np.sin(x)*np.pi/8
y2 =np.cos(x)*np.pi/4

# Loop through simulation steps
for i in range(duration):
    # Control each leg joint individually using the sine wave function `y[i]`
    ps.Set_Motor_For_Joint(bodyIndex=robotId, 
                           jointName=b'Leg1_Body',  # Control first leg joint
                           controlMode=p.POSITION_CONTROL, 
                           targetPosition=y[i], 
                           maxForce=500)
    
    ps.Set_Motor_For_Joint(bodyIndex=robotId, 
                           jointName=b'Leg2_Body',  # Control second leg joint
                           controlMode=p.POSITION_CONTROL, 
                           targetPosition=y[i], 
                           maxForce=500)
    
    ps.Set_Motor_For_Joint(bodyIndex=robotId, 
                           jointName=b'Leg3_Body',  # Control third leg joint
                           controlMode=p.POSITION_CONTROL, 
                           targetPosition=-y[i], 
                           maxForce=500)

    ps.Set_Motor_For_Joint(bodyIndex=robotId, 
                           jointName=b'Leg4_Body',  # Control third leg joint
                           controlMode=p.POSITION_CONTROL, 
                           targetPosition=-y[i], 
                           maxForce=500)
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1 / 500)  # Control loop timing (adjust to desired simulation speed)

# Disconnect from simulation
p.disconnect()

# for i in range(duration):
#     ps.Set_Motor_For_Joint(bodyIndex = robotId, 
#                            jointName = b'Foot_Torso',
#                            controlMode = p.POSITION_CONTROL,
#                            targetPosition = y[i],
#                            maxForce = 500)
#     p.stepSimulation()
#     time.sleep(1/500)

# p.disconnect()