import numpy as np
import sympy as sp

# Define symbolic variables
t1, t2, t3 = sp.symbols('g b a')

# Define the rotation matrices
Rotx = sp.Matrix([[1,0,0],[0,sp.cos(t1),-sp.sin(t1)],[0,sp.sin(t1),sp.cos(t1)]])
Roty = sp.Matrix([[sp.cos(t2),0,sp.sin(t2)],[0,1,0],[-sp.sin(t2),0,sp.cos(t2)]])
Rotz = sp.Matrix([[sp.cos(t3),-sp.sin(t3),0],[sp.sin(t3),sp.cos(t3),0],[0,0,1]])

result = Rotz@Roty@Rotx
# Simplify the result
simplified_result = sp.simplify(result)

# Display the simplified result
print(simplified_result)


r11 = .6
r12 = .79
r13 = -.01
r21 = .47
r22 = -.34
r23 = .81
r31 = .64
r32 = -.5
r33 = -.58
# Calculate the angle theta and axis of rotation
Rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

equations = []
for i in range(3):
    for j in range(3):
        equations.append(sp.Eq(simplified_result[i, j], Rot[i, j]))

    # Step 5: Solve the equations
solution = sp.solve(equations, (t1, t2, t3))
print('solution: ',solution) 