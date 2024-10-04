import numpy as np 
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

r11 =0.866
r12 = -.5
r13= 0
r21=.5
r22=.866
r23=0
r31=0
r32=0
r33=1
Rot = np.array([[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]])

# print the final rotation matrix
print(Rot)
original = np.array([[1,0,0],[0,1,0],[0,0,1]])

origin = np.zeros(3)
x0 = np.transpose(np.array([1,0,0]))
y0 = np.transpose(np.array([0,1,0]))
z0 = np.transpose(np.array([0,0,1]))
x_axis = np.matmul(Rot,x0)
y_axis = np.matmul(Rot,y0)
z_axis = np.matmul(Rot,z0)
print(x_axis)
print(y_axis)
print(z_axis)

# Visualize the coordinate frame
fig = plt.figure()

ax = fig.add_subplot(211, projection='3d')
ax.quiver(*origin, *x_axis, color='r', label='X-axis')
ax.quiver(*origin, *y_axis, color='g', label='Y-axis')
ax.quiver(*origin, *z_axis, color='b', label='Z-axis')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

ax = fig.add_subplot(212, projection='3d')
ax.quiver(*origin, *x0, color='r', label='X0')
ax.quiver(*origin, *y0, color='g', label='Y0')
ax.quiver(*origin, *z0, color='b', label='Z0')

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()