# Jake Little

# Rotation Matrix Check
# A rotation matrix will always be a square matrix.
# As a rotation matrix is always an orthogonal matrix the transpose will be equal to the inverse of the matrix.
# The determinant of a rotation matrix will always be equal to 1.
# Multiplication of rotation matrices will result in a rotation matrix.
# If we take the cross product of two rows of a rotation matrix it will be equal to the third.
# The dot product of a row with a column of a rotation matrix will be equal to 1.

import numpy as np
from numpy.linalg import inv , det

# Insert matrix to check if it is a rotation matrix
theta = np.deg2rad(30)
rot_x_theta = np.array([[ 1, 0, 0],
                        [ 0, np.cos(theta), -np.sin(theta)],
                        [ 0,np.sin(theta), np.cos(theta)]]) 
rMat = rot_x_theta

# Below is NOT a rotation matrix, comment out if youd like to use real one
# Will return errors on purpose
# rMat = np.array([[ 1, 0, 0],
#                         [ 0, 0, -1],
#                         [ 0,0, 5]]) 
print(rMat.shape)
identity = np.identity(3)

try: 
    size = rMat.shape
    if size == (3, 3):
        print("Square!")
        invR = inv(rMat)
        print("Inverse", invR)
        rMatTransp = np.transpose(rMat)
        print("Transpose", np.transpose(rMat))
        if np.isclose(invR,rMatTransp,rtol=1e-5,atol=1e-8,equal_nan=False).all:
            print("Transpose is the inverse!")
            if det(rMat) == 1 and det(rMatTransp) == 1:
                print("Determinate of matrix and its transpose/inverse is 1")
                print("This is a rotation matrix. Yippie!")
            else:
                print("Determinates not equal to 1")
        else:
            print("Inverse is not the transpose")
    else:
        print("Not a 3x3 square matrix")
                
                

except: 
    print("Failed, not a rotation matrix")
    