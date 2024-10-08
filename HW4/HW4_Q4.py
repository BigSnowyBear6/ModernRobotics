import numpy as np
import sympy as sp

t1, t2, t3 = sp.symbols('g1 b1 a1')

# Define the rotation matrices
Rotx = sp.Matrix([[1,0,0],[0,sp.cos(t1),-sp.sin(t1)],[0,sp.sin(t1),sp.cos(t1)]])
Roty = sp.Matrix([[sp.cos(t2),0,sp.sin(t2)],[0,1,0],[-sp.sin(t2),0,sp.cos(t2)]])
Rotz = sp.Matrix([[sp.cos(t3),-sp.sin(t3),0],[sp.sin(t3),sp.cos(t3),0],[0,0,1]])

result = Rotz@Roty@Rotx
# Simplify the result
simplified_result = sp.simplify(result)
print('Result RPY')
print(simplified_result)

# Define symbolic variables
t4, t5, t6 = sp.symbols('g2 b2 a2')

# Define the rotation matrices
Rotz1 = sp.Matrix([[sp.cos(t4),-sp.sin(t4),0],[sp.sin(t4),sp.cos(t4),0],[0,0,1]])
Roty1 = sp.Matrix([[sp.cos(t5),0,sp.sin(t5)],[0,1,0],[-sp.sin(t5),0,sp.cos(t5)]])
Rotz2 = sp.Matrix([[sp.cos(t6),-sp.sin(t6),0],[sp.sin(t6),sp.cos(t6),0],[0,0,1]])

result2 = Rotz2@Roty1@Rotz1

# Simplify the result
simplified_result2 = sp.simplify(result2)

# Display the simplified result
print('Result ZYZ')
print(simplified_result2)

Arpy = np.deg2rad(30)
Brpy =np.deg2rad(60)
Grpy = np.deg2rad(45)

Rotx = np.array([[1,0,0],[0,np.cos(Arpy),-np.sin(Arpy)],[0,np.sin(Arpy),np.cos(Arpy)]])
Roty = np.array([[np.cos(Brpy),0,np.sin(Brpy)],[0,1,0],[-np.sin(Brpy),0,np.cos(Brpy)]])
Rotz = np.array([[np.cos(Grpy),-np.sin(Grpy),0],[np.sin(Grpy),np.cos(Grpy),0],[0,0,1]])
Rot = Rotz@Roty@Rotx

Bsin = np.sqrt((Rot[0,2]*Rot[0,2]) + (Rot[1,2]*Rot[1,2]))
print('Bsin: ',Bsin)

Bzyz = np.atan2(Bsin,Rot[2,2])
Azyz = np.atan2(Rot[1,2]/Bsin,Rot[0,2] /Bsin)
Gzyz = np.atan2(Rot[2,1]/Bsin,-Rot[2,0]/Bsin)

Bsin = -Bsin
Bzyz2 = np.atan2(Bsin,Rot[2,2])
Azyz2 = np.atan2(Rot[1,2]/Bsin,Rot[0,2]/Bsin)
Gzyz2 = np.atan2(Rot[2,1]/Bsin,-Rot[2,0]/Bsin)

# Print angle sets
print('First angle set (in degress):', np.rad2deg([Azyz, Bzyz, Gzyz]))
print('Second angle set (in degrees):', np.rad2deg([Azyz2, Bzyz2, Gzyz2]))