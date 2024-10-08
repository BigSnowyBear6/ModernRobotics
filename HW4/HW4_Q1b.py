import numpy as np
import matplotlib.pyplot as plt

w = np.array([1,2,0])
angle = np.linalg.norm(w)
wHat =w/angle
[a1,a2,a3] = wHat
wMat = np.array([[0,-a3,a2],[a3,0,-a1],[-a2,a1,0]]) # Skew symmetric matrix
print(wMat)

R = np.eye(3) + np.sin(angle)*(wMat) + (1-np.cos(angle))*(wMat@wMat)
print('Rotation Matrix: ',R)

origin = np.zeros(3)
x0 = np.transpose(np.array([1,0,0]))
y0 = np.transpose(np.array([0,1,0]))
z0 = np.transpose(np.array([0,0,1]))

x1 = R@x0
y1 = R@y0
z1 = R@z0
# Visualize the coordinate frame
fig = plt.figure()
ax = fig.add_subplot(211, projection='3d')

ax.quiver(*origin, *x0, color='r', label='X0')
ax.quiver(*origin, *y0, color='g', label='Y0')
ax.quiver(*origin, *z0, color='b', label='Z0')
ax.quiver(*origin, *np.array([wHat[0].item(),wHat[1].item(),wHat[2].item()]), color='orange', label='rotation axis')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

ax = fig.add_subplot(212, projection='3d')

ax.quiver(*origin, *x1, color='r', label='X1')
ax.quiver(*origin, *y1, color='g', label='Y1')
ax.quiver(*origin, *z1, color='b', label='Z1')
ax.quiver(*origin, *np.array([wHat[0].item(),wHat[1].item(),wHat[2].item()]), color='orange', label='rotation axis')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()