# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.นครินทร์_6533
2.พีรดา_6573
'''
#=============================================<Import library>==================================================#
from HW3_utils import FKHW3
import numpy as np
#===============================================<คำตอบข้อ 1>=====================================================#
def endEffectorJacobianHW3(q:list[float])->list[float]:
    # Get q1,q2 and q3 from argument "q"
    [q1,q2,q3] = q
    
    # Get R, P, R_e, p_e from FKHW3(q)
    R, P, R_e, p_e = FKHW3(q)
    
    # Define position vector from FKHW3(q)
    p_0_1 = np.array(P[:,0]).reshape(3,1)  # Position vector from frame 0 to frame 1
    p_0_2 = np.array(P[:,1]).reshape(3,1)  # Position vector from frame 0 to frame 2
    p_0_3 = np.array(P[:,2]).reshape(3,1)  # Position vector from frame 0 to frame 3
    p_0_e = np.array(P[:,3]).reshape(3,1)  # Position vector from frame 0 to frame e
    
    # Define rotation axis variables
    '''
        z_1 , z_2 , z_3 are from projection of z at frame 1, 2, 3 to frame 0
        ** You can see how we calculate at calculationpaper.pdf
    '''
    z_1 = np.array([[0], [0], [1]])                    # Rotation axis at joint 1
    z_2 = np.array([[np.sin(q1)], [np.cos(q1)], [0]])  # Rotation axis at joint 2
    z_3 = np.array([[np.sin(q1)], [np.cos(q1)], [0]])  # Rotation axis at joint 3
    
    '''
        Jacobian calculation
            J_i = [Jv_i(q) Jw_i(q)]^T
            Jv(q)|Jw(q) = [z_i 0]^T if joint i is Prismatic joint
            Jv(q)|Jw(q) = [z_ix(P_0_e - P_0_i) 0]^T if joint i is Revolute joint
            Let ^T is transpose
            ** You can see how we calculate at calculationpaper.pdf
    '''
    try:
        # Compute the cross product of z and p using np.cross
        cross_1 = np.cross(z_1.flatten(), (p_0_e - p_0_1).flatten())
        cross_2 = np.cross(z_2.flatten(), (p_0_e - p_0_2).flatten())
        cross_3 = np.cross(z_3.flatten(), (p_0_e - p_0_3).flatten())
    except Exception as e:
        print(f"Error calculating cross product: {e}")
        return np.zeros((3, 6))
        
    # Reshape the cross product from 1x3 to 3x1
    cross_1 = cross_1.reshape(3, 1)
    cross_2 = cross_2.reshape(3, 1)
    cross_3 = cross_3.reshape(3, 1)
    
    # Concatenate the cross product at vertical axis
    column_1 = np.concatenate((cross_1, z_1), axis = 0)
    column_2 = np.concatenate((cross_2, z_2), axis = 0) 
    column_3 = np.concatenate((cross_3, z_3), axis = 0) 
    # column_1 = np.concatenate((j1, z_1), axis = 0)
    # column_2 = np.concatenate((j2, z_2), axis = 0) 
    # column_3 = np.concatenate((j3, z_3), axis = 0) 
    
    # Stack the cross product at horizontal axis
    jacobian = np.hstack((column_1, column_2, column_3))
    
    # Construct the block diagonal transformation matrix
    T = np.block([
        [R_e.T, np.zeros((3, 3))],
        [np.zeros((3, 3)), R_e.T]
    ])
    
    jacobian = T @ jacobian
    
    return jacobian
#===============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
def checkSingularityHW3(q:list[float])->bool:
    '''
        Check singularity
            Singularity is happend when det(J) is 0
            But we can calculate det if matrix is square. So we need to simplify the matrix to be square.
            From robot model end effector can only translate in x, y, z direction but can not rotate. So we will reduce the matrix by removing the Angular Jacobian.
            
            J_i = [Jv_i(q)] ; Remove the Angular Jacobian
            
            ** In computer programming or real world when we calculate Det(J) and it near to zero with small tolerance we can let's it be singularity
    '''
    try:
        det_J = np.linalg.det(endEffectorJacobianHW3(q)[0:3, :])
        print(f"determinant of Jacobian from checkSingularityHW3 function is {det_J}")
    except Exception as e:
        print(f"Error calculating det: {e}")
        return False
    tolerance = 1e-3
    
    return np.isclose(det_J, 0.0, atol=tolerance).all()
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    # Get R, P, R_e, p_e from FKHW3(q)
    R, P, R_e, p_e = FKHW3(q)
    
    # w is a list of [forces , moments]
    F_e = np.array(w)[0:3].reshape(3, 1)
    M_e = np.array(w)[3:6].reshape(3, 1)
    
    # Create new w
    w = np.vstack((F_e, M_e))
    
    # Calculate the tau
    '''
        tau calculation
            tau = Jacobian(q)^T @ w
            **  w must be reference same as the jacobian like if jacobian is reference from frame 0 w must be reference from frame 0
                if it not from the same frame we need to change frame of w to reference frame same as jacobian
    '''
    tau = endEffectorJacobianHW3(q).T @ w
    
    return tau
#==============================================================================================================