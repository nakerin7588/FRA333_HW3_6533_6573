# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.นครินทร์_6533
2.พีรดา_6573
'''

#=============================================<Import library>==================================================#
from HW3_utils import FKHW3
from FRA333_HW3_6533_6573 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3

import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import SE3, SO3
import numpy as np
#=========================<Create robot from roboticstoolbox for check answer>=================================#
d1, a2 = 0.0892, 0.425
mdh = [[0, 0, d1, np.pi], [0, np.pi/2.0, 0, 0], [-a2, 0, 0, 0]]
revjoint = [] # Create revolute joint
for data in mdh:
    revjoint.append(rtb.RevoluteMDH(a=data[0], alpha=data[1], d=data[2], offset=data[3])) # Append revolute joint

# Get Transformation Matrix from FKHW3 function
R,P,R_e,p_e = FKHW3([0, 0 ,0]) # Home configuration
T_0_3 = np.hstack((R[:, :, 2], P[:, 2].reshape(3, 1)))
T_0_3 = np.vstack((T_0_3, [0, 0, 0, 1]))
T_0_3 = SE3(T_0_3)

T_0_e = np.hstack((R[:, :, 3], P[:, 3].reshape(3, 1)))
T_0_e = np.vstack((T_0_e, [0, 0, 0, 1]))
T_0_e = SE3(T_0_e)

T_3_e = T_0_3.inv() @ T_0_e

# Create robot model from MDH-Parameter 
robot = rtb.DHRobot(
    [
        revjoint[0],
        revjoint[1],
        revjoint[2]
    ],tool = T_3_e, name="3R robot"
    )

#=====================================<Define important variables>==============================================#
q = [np.pi, np.pi, np.pi] # Configuration space ** Unit must be radians
w = [1, 2, 3, 4, 5, 6] # Force and Moment that reference at frame e [force moment]
R,P,R_e,p_e = FKHW3(q) # Get data from FKHW3 with new q
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
print("================================")
print("Question 1 start answer checking")
jacobian = endEffectorJacobianHW3(q) # Calculate Jacobian from endEffectorJacobianHW3

# Check answer
print("Answer from endEffectorJacobianHW3")
print(jacobian)
print("Answer from RoboticsToolbox")
print(robot.jacob0(q = q)) # Get Jacobian from roboticsToolbox
# Use np.isclose to check that the Jacobian from endEffectorJacobianHW3 and the Jacobian from roboticsToolbox are the same with low tolerance
print(f"Answer from np.isclose is {np.isclose(jacobian, robot.jacob0(q = q), atol=1e-3).all()}")
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
print("================================")
print("Question 2 start answer checking")

# Check answer
print("Answer from endEffectorJacobianHW3")
flag = checkSingularityHW3(q) # Check singularity from checkSingularityHW3
print(flag)
print("Answer from RoboticsToolbox")
print(f"determinant of Jacobian from checkSingularityHW3 function is {np.linalg.det(robot.jacob0(q = q)[0:3, :])}")
print(np.isclose(np.linalg.det(robot.jacob0(q = q)[0:3, :]), 0.0, atol=1e-3).all()) # Get Jacobian from roboticsToolbox and reduce it to be square
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
print("================================")
print("Question 3 start answer checking")
tau = computeEffortHW3(q, w)

# Check answer
print("Answer from computeEffortHW3")
print(tau)
print("Answer from RoboticsToolbox")
print("Check answer from jacobian that reference from frame 0")
w_reframe = np.vstack((R_e @ (np.array(w)[0:3].reshape(3, 1)), R_e @ (np.array(w)[3:6].reshape(3, 1)))) # w that has rereference frame to frame 0 same as jacobian
tau_jacob0 = robot.jacob0(q = q).T @ w_reframe # Calculate tau using jacobian from roboticsToolbox
print(tau_jacob0) # Calculate tau using jacobian from roboticsToolbox
print("Check answer from jacobian that reference from frame e")
tau_jacobe = robot.jacobe(q = q).T @ np.array(w).reshape(6,1)
print(tau_jacobe) # Calculate tau using jacobian from roboticsToolbox
# Use np.isclose to check that the tau from computeEffortHW3 and the computeEffortHW3 from roboticsToolbox are the same with low tolerance
print(f"Answer from np.isclose(RTB reference jacobian from frame 0) is {np.isclose(tau, tau_jacob0, atol=1e-3).all()}")
print(f"Answer from np.isclose(RTB reference jacobian from frame e) is {np.isclose(tau, tau_jacobe, atol=1e-3).all()}")
#==============================================================================================================#