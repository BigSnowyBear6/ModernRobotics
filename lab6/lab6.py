# -*- coding: utf-8 -*-
"""lab6.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/1iNJRQyfXY-UVcPmaQBibUlZHVcwGjV0w
"""

#Question1. step1
import numpy as np

np.set_printoptions(suppress=True)

# Angles in radians
theta_1 = 90 * np.pi / 180
theta_2 = -45 * np.pi / 180
theta_3 = 0
theta_4 = 45 * np.pi / 180

# Skew-symmetric matrices
a1 = 0
a2 = 0
a3 = 1
z_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])
a1 = 0
a2 = 1
a3 = 0
y_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])

# Calculate the rotation matrices using Rodrigue's formula
e_z_hat_bracket_theta_1 = np.eye(3) + np.sin(theta_1) * z_hat_bracket + (1 - np.cos(theta_1)) * np.matmul(z_hat_bracket, z_hat_bracket)
e_y_hat_bracket_theta_2 = np.eye(3) + np.sin(theta_2) * y_hat_bracket + (1 - np.cos(theta_2)) * np.matmul(y_hat_bracket, y_hat_bracket)
e_y_hat_bracket_theta_3 = np.eye(3) + np.sin(theta_3) * y_hat_bracket + (1 - np.cos(theta_3)) * np.matmul(y_hat_bracket, y_hat_bracket)
e_y_hat_bracket_theta_4 = np.eye(3) + np.sin(theta_4) * y_hat_bracket + (1 - np.cos(theta_4)) * np.matmul(y_hat_bracket, y_hat_bracket)

# Calculate the final rotation matrix product of exponentials
R = np.matmul(np.matmul(np.matmul(np.matmul(np.eye(3), e_z_hat_bracket_theta_1), e_y_hat_bracket_theta_2), e_y_hat_bracket_theta_3), e_y_hat_bracket_theta_4)

# Print the final rotation matrix
print(R)

# Question: Is this familiar to you?
Yes

# Question1.step2
import sympy as sp
import numpy as np
from sympy import solve

# Define symbolic variables
t1, t2, t3, t4 = sp.symbols('t1 t2 t3 t4')

# Define the skew-symmetric matrices
a1 = 0
a2 = 0
a3 = 1
z_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])
a1 = 0
a2 = 1
a3 = 0
y_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])

# Calculate e_bracket_z_t1
e_bracket_z_t1 = sp.eye(3) + sp.sin(t1)*z_hat_bracket+(1-sp.cos(t1))*sp.Matrix(z_hat_bracket)*sp.Matrix(z_hat_bracket)

# Calculate e_bracket_y_t2
e_bracket_y_t2 = sp.eye(3) + sp.sin(t2)*y_hat_bracket+(1-sp.cos(t2))*sp.Matrix(y_hat_bracket)*sp.Matrix(y_hat_bracket)

# Calculate e_bracket_y_t3
e_bracket_y_t3 = sp.eye(3) + sp.sin(t3)*y_hat_bracket+(1-sp.cos(t3))*sp.Matrix(y_hat_bracket)*sp.Matrix(y_hat_bracket)

# Calculate e_bracket_y_t4
e_bracket_y_t4 = sp.eye(3) + sp.sin(t4)*y_hat_bracket+(1-sp.cos(t4))*sp.Matrix(y_hat_bracket)*sp.Matrix(y_hat_bracket)

# Calculate the final result
result = e_bracket_z_t1 @ e_bracket_y_t2 @ e_bracket_y_t3 @ e_bracket_y_t4

# Simplify the result
simplified_result = sp.simplify(result)

# Display the simplified result
print(simplified_result)

# Question1.step2
import sympy as sp
import numpy as np
from sympy import solve

# Define symbolic variables
t1, t2, t3 = sp.symbols('t1 t2 t3')

# Define the skew-symmetric matrices
a1 = 1
a2 = 0
a3 = 0
x_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])
a1 = 0
a2 = 0
a3 = 1
z_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])
a1 = 0
a2 = 1
a3 = 0
y_hat_bracket = np.array([[0, -a3, a2], [a3, 0, -a1], [-a2, a1, 0]])

# Calculate e_bracket_z_t1
e_bracket_z_t1 = sp.eye(3) + sp.sin(t1)*z_hat_bracket+(1-sp.cos(t1))*sp.Matrix(z_hat_bracket)*sp.Matrix(z_hat_bracket)

# Calculate e_bracket_y_t2
e_bracket_y_t2 = sp.eye(3) + sp.sin(t2)*y_hat_bracket+(1-sp.cos(t2))*sp.Matrix(y_hat_bracket)*sp.Matrix(y_hat_bracket)

# Calculate e_bracket_y_t3
e_bracket_x_t3 = sp.eye(3) + sp.sin(t3)*x_hat_bracket+(1-sp.cos(t3))*sp.Matrix(x_hat_bracket)*sp.Matrix(x_hat_bracket)


# Calculate the final result
result = e_bracket_z_t1 @ e_bracket_y_t2 @ e_bracket_x_t3

# Simplify the result
simplified_result = sp.simplify(result)

# Display the simplified result
print(simplified_result)