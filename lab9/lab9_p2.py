# -*- coding: utf-8 -*-
"""lab9.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/1vXFAyJERWg7Dijt2IHwXTp0qCGdFk5wZ
"""

import numpy as np

def POE(q,a,rot,jt):
  T = np.eye(4,4)
  n = len(q)
  for ii in range(n-1,-1,-1):
    if jt[ii] == 'R':
      rot_hat = np.array([[0, -rot[ii][2], rot[ii][1]],
            [rot[ii][2], 0, -rot[ii][0]],
            [-rot[ii][1], rot[ii][0], 0]])
      e_rot_hat=np.eye(3,3)+rot_hat*np.sin(q[ii])+rot_hat@rot_hat*(1-np.cos(q[ii]))
    elif jt[ii] == 'P':
      rot_hat = np.zeros((3,3))
      e_rot_hat = np.eye(3,3)

    if jt[ii] == 'R' and ii>0:
      Sv = -np.transpose(np.cross(rot[ii][:], a[ii][:]))
    elif jt[ii] == 'R' and ii==0:
      Sv = [0, 0, 0]
    elif jt[ii] == 'P':
      Sv = np.transpose(a[ii][:]).reshape((3, 1))

    p = (np.eye((3))*q[ii]+(1-np.cos(q[ii]))*rot_hat+(q[ii]-np.sin(q[ii]))*rot_hat@rot_hat)@Sv
    p = p.reshape(3,1)
    e_zai = np.block([[e_rot_hat, p], [0, 0, 0, 1]])

    T = e_zai@T
  return T

def FK_PoE(q,a,rot,jt,M):
  T=POE(q,a,rot,jt)
  Tf=T@M
  print("Tf", Tf)
  R=Tf[0:3,0:3]
  p=Tf[0:3,3]
  return R, p


t1 = 0
t2 = 0
t3 = -np.pi/2
t4 = np.pi/2
#  Might need the end effector included as prismatic

theta = [t1,t2,t3,t4]
H1 = 89.45/1000
H2 = 100/1000
L1 = 35/1000
L2 = 100/1000
L3 = 107.6/1000

a = [[0,0,0],[0,0,H1],[L1,0,H1+H2],[L1+L2,0,H1+H2]]
rot = [[0,0,1],[0,1,0],[0,1,0],[0,1,0]]
jt = 'RRRR' # Maybe R at end
M = [[1,0,0,L1+L2+L3],[0,1,0,0],[0,0,1,H1+H2],[0,0,0,1]]

R,p=FK_PoE(theta,a,rot,jt,M)
print("R is", np.round(R))
print("p is", p)
