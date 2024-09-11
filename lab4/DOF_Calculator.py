## Degree of Freedom Calculator
# Jake Little & Guangping Liu

# Using Grubler's Formula


import numpy as np

m = input("Type 3D or 2D: ")
N = int(input("Number of links including ground: "))
J = int(input("Number of joints: "))
f = [0] * J  # Initialize the list with zeros instead of empty strings

for i in range(J):
    f[i] = int(input(f"DOF for joint {i + 1}: "))  # Convert input to integer

F = np.sum(f)  # Sum the list using NumPy

match m:
    case '3D':
         m = 6
    case '2D':
        m = 3
    case other:
        print('Nothing')

dof = m*(N-1-J)+F
print(F," degrees of freedom")