import numpy as np 
import matplotlib.pyplot as plt
np.set_printoptions(suppress=True)

# angles in radians
theta_1 = np.deg2rad(90)
theta_2 = np.deg2rad(-45)
theta_3 = np.deg2rad(0)
theta_4 = np.deg2rad(45)

rot_z_theta_1 = np.array([[ np.cos(theta_1), -np.sin(theta_1),0 ],
                        [ np.sin(theta_1), np.cos(theta_1), 0],
                        [ 0, 0, 1]]) 

rot_y_theta_2 = np.array([[ np.cos(theta_2), 0, np.sin(theta_2)],
                        [ 0, 1, 0],
                        [ -np.sin(theta_2), 0, np.cos(theta_2)]]) 
rot_y_theta_3 = np.array([[ np.cos(theta_3), 0, np.sin(theta_3)],
                        [ 0, 1, 0],
                        [ -np.sin(theta_3), 0, np.cos(theta_3)]]) 
rot_y_theta_4 = np.array([[ np.cos(theta_4), 0, np.sin(theta_4)],
                        [ 0, 1, 0],
                        [ -np.sin(theta_4), 0, np.cos(theta_4)]]) 
eye = np.array([[ 1, 0, 0],
                        [ 0, 1, 0],
                        [ 0, 0, 1]]) 

# tool frame w.r.t the base frame

rot_0_5 = np.matmul(np.matmul(np.matmul(np.matmul(eye,rot_z_theta_1),rot_y_theta_2),rot_y_theta_3),rot_y_theta_4)

# print the final rotation matrix
print(rot_0_5)

origin = np.zeros(3)
x0 = np.transpose(np.array([1,0,0]))
y0 = np.transpose(np.array([0,1,0]))
z0 = np.transpose(np.array([0,0,1]))
x_axis = np.matmul(rot_0_5,x0)
y_axis = np.matmul(rot_0_5,y0)
z_axis = np.matmul(rot_0_5,z0)
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
