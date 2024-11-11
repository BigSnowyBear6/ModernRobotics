% Prismatic POE test
t1 = deg2rad(45); t2 = deg2rad(15); t3 = deg2rad(-20); t4 = deg2rad(90); t5 = deg2rad(30); t6 = deg2rad(-10);


theta = [t1;t2;t3;t4;t5;t6];
L1 = 290/1000;
L2 = 400/1000;
L3 = 315/1000;
L4 = 170/1000;


a = [0 0 0;0 0 L1;0 0 L1+L2;0 0 L1+L2+L3;0 0 L1+L2+L3;0 0 L1+L2+L3+L4];
rot = [0 0 1;0 1 0;0 -1 0;0 0 -1;0 -1 0;0 0 -1];
jt = 'RRRRRR';
M = [-1 0 0 0;0 -1 0 0;0 0 1 L1+L2+L3+L4;0 0 0 1];
[R,p]=FK_PoE(theta,a,rot,jt,M)