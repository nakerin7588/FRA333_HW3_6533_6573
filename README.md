# FRA333_HW3_6533_6573
This is README file for explain how to check the answer of FRA333_HW3_6533_6573.

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
## Question 1: Compute the endeffector jacobian