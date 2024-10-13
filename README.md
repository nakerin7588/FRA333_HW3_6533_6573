# FRA333_HW3_6533_6573
This is README file for explain how to check the answer of FRA333_HW3_6533_6573.

Members:
1. นายนครินทร์ เจตนาธรรมจิต    65340500033
2. นางสาวพีรดา สุขถาวร        65340500073

# Getting Started
To check the answer of FRA333_HW3_6533_6573. You must have all of prerequisites for FRA333_HW3_6533_6573.
## Prerequisites
⚠️ **Warning:** Make sure you have python version >= 3.6 already
* numpy
    ```
    pip3 install numpy==1.24.1
    ```
* robotics toolbox
    ```
    pip3 install roboticstoolbox-python
    ```
## Installation
Follow the command below to dowload and install package.
1. Clone the repository
    ```
    git clone https://github.com/nakerin7588/FRA333_HW3_6533_6573.git
    ```

# Usage
At `testScript.py` you can run this file to check the answer of the function in `FRA333_HW3_6533_6573.py`. And if you want to test the answer of the function with your specified arguments `q for configuration space` and `w for wrench` at `Define important variables` section. After that you can run this file again for check the answer of the function.

Define important variables section is look like this in test.py:
```Python
#=====================================<Define important variables>==============================================#
q = [np.pi, np.pi, np.pi] # Configuration space ** Unit must be radians
w = [1, 2, 3, 4, 5, 6] # Force and Moment that reference at frame e [force moment]
R,P,R_e,p_e = FKHW3(q) # Get data from FKHW3 with new q
```
**Important:** The unit of the variables are defined in `radians for q` , `N for force` and `N.m for moment`. `q` variable has specific meaning at idex start with 0 to 2 that mean rotation at joint in order 1, 2, 3. `w` variable has specific meaning at index. Index `0, 1, 2` is for force in x, y, z axis. Index `3, 4, 5` is for moment in x, y, z axis.

* **q** = [q1, q2, q3]
* **w** = [fx, fy, fz, mx, my, mz]

# How to check answer
**This answer in terminal are set the q to [pi, pi, pi] and the w to [1, 2, 3, 4, 5, 6]**
## Question 1: Compute the endeffector jacobian
To check answer of this question you will need to define the `q parameter` from Define important variables first. After that you can run the `testScript.py` to check the answer. The answer will show like this in terminal:

```
Question 1 start answer checking
Answer from endEffectorJacobianHW3
[[ 4.94300000e-02 -3.45264670e-17 -8.65739582e-17]
 [ 1.39695452e-17 -4.94300000e-02 -4.74430000e-01]
 [-1.09000000e-01 -9.30000000e-02 -9.30000000e-02]
 [ 1.22464685e-16  1.00000000e+00  1.00000000e+00]
 [ 1.00000000e+00 -1.22464685e-16 -1.22464685e-16]
 [ 1.83697015e-16 -3.06161695e-16 -3.06161695e-16]]

Answer from RoboticsToolbox
[[ 4.94300000e-02  2.66789317e-18 -2.33558524e-17]
 [ 1.39695456e-17 -4.94300000e-02 -4.74430000e-01]
 [-1.09000000e-01 -9.30000000e-02 -9.30000000e-02]
 [ 1.22464683e-16  1.00000000e+00  1.00000000e+00]
 [ 1.00000000e+00 -6.12323426e-17 -6.12323426e-17]
 [ 1.83697017e-16  6.12323426e-17  6.12323426e-17]]

Answer from np.isclose is True
```

Section 1 is the output from **endEffectorJacobianHW3**. Section 2 is the output from **roboticstoolbox**. Section 3 is the output from **np.isclose** that check is output from **endEffectorJacobianHW3 same as roboticstoolbox**.

**You can check the source code for more details.** At the `def endEffectorJacobianHW3(q:list[float])->list[float]`

## Question 2: Check singularity
To check answer of this question you will need to define the `q parameter` from Define important variables first. After that you can run the `testScript.py` to check the answer. The answer will show like this in terminal:

```
Question 2 start answer checking
Answer from checkSingularityHW3
determinant of Jacobian from checkSingularityHW3 function is -0.0019537207499999987
Is this configuraion space make robot singularity: False

Answer from RoboticsToolbox
determinant of Jacobian(ref frame 0) from checkSingularityHW3 function is -0.001953720749999997
Is this configuraion space make robot singularity: False
determinant of Jacobian(ref frame e) from checkSingularityHW3 function is -0.001953720749999997
Is this configuraion space make robot singularity: False
```

Section 1 is the output from checkSingularityHW3 that show det(J) and **check the singularity if True is singularity if False is not singularity**. Section 2 is the output from roboticstoolbox that show det(J) and **check the singularity if True is singularity if False is not singularity**. **From roboticsstoolbox provide 2 jacobian(reference from frame 0 and frame e)**

## Question 3: Conpute effort
To check answer of this question you will need to define the `q parameter` and `w parameter` from Define important variables first. After that you can run the `testScript.py` to check the answer. The answer will show like this in terminal:

```
Question 3 start answer checking
Answer from computeEffortHW3
[[4.72243]
 [3.62214]
 [2.77214]]

Answer from RoboticsToolbox
Check answer from jacobian that reference from frame 0
[[4.72243]
 [3.62214]
 [2.77214]]
Answer from np.isclose(RTB reference jacobian from frame 0) is True
Check answer from jacobian that reference from frame e
[[4.72243]
 [3.62214]
 [2.77214]]
Answer from np.isclose(RTB reference jacobian from frame e) is True
```

Section 1 is the output from computeEffortHW3. Section 2 is the output from roboticstoolbox. To cheack is answer from computeEffortHW3 is correct. We use **np.isclose** for check that answer from **computeEffortHW3 is same as answer from roboticsstoolbox**. **From roboticsstoolbox provide 2 jacobian(reference from frame 0 and frame e)**

> **To see more calculation details, you can read calculationpaper.pdf**