import numpy as np
import sympy as sp
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True)
# Input the rotation matrix
r11 = 0
r12 =-1
r13 =0
r21 =0
r22 =0
r23 =-1
r31 =1
r32 =0
r33 =0

a1,a2,a3 = sp.symbols('a1 a2 a3')
wMAt = np.array([[0,-a3,a2],[a3,0,-a1],[-a2,a1,0]]) # Skew symmetric matrix
wMAt2 = wMAt@wMAt
print(wMAt2)

# Calculate the angle theta and axis of rotation
Rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

theta = np.arccos((r11 + r22 + r33 -1)/2)
print('theta: ',theta)
print('sin(theta)= ',np.sin(theta))
print(np.mod(theta,2*np.pi))
if np.isclose(np.sin(theta),0,rtol=1e-2):
    if np.isclose(np.mod(theta,2*np.pi),0,rtol=1e-2): # even integer of pi
        print('Axis undefined, even pi integer')
    else: # odd integer of pi
        # R = I + 2[w]^2
        print('odd integer multiple of pi')
        w = (Rot - np.transpose(Rot))/(2*np.sin(theta))
        what2 = (Rot-np.identity(3))/2
        equations = []
        for i in range(3):
            for j in range(3):
                equations.append(sp.Eq(wMAt2[i, j], what2[i, j]))

        # Step 5: Solve the equations
        solution = sp.solve(equations, (a1, a2, a3))
        wVect = np.array([a1,a2,a3])
        print('solution: ',wVect) 
else:
    # R=e^[ω]θ
    print('Not integer multiple of pi')
    w = (Rot - np.transpose(Rot))/(2*np.sin(theta))
    print('w (skew-symmetric matrix)= ',w)
    wVect = np.array([w[2,1].item(),w[0,2].item(),w[2,0].item()])
    print('w (vector): ', wVect)
    



#  Plotting


origin = np.zeros(3)
x0 = np.transpose(np.array([1,0,0]))
y0 = np.transpose(np.array([0,1,0]))
z0 = np.transpose(np.array([0,0,1]))

# Visualize the coordinate frame
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.quiver(*origin, *x0, color='r', label='X0')
ax.quiver(*origin, *y0, color='g', label='Y0')
ax.quiver(*origin, *z0, color='b', label='Z0')
ax.quiver(*origin, *np.array([w[0,2].item(),w[1,0].item(),w[2,1].item()]), color='orange', label='rotation axis')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()