# Jacobian
import numpy as np

t1 = self.angles[0]
t2 = self.angles[1]
t3 = self.angles[2]
t4 = self.angles[3]
J11 = - 35*np.cos(t2)*np.sin(t1)+ 100*np.sin(t1)*np.sin(t2) - 100*np.cos(t2)*np.cos(t3)*np.sin(t1)+ 100*np.sin(t1)*np.sin(t2)*np.sin(t3) - (538*np.cos(t2)*np.cos(t3)*np.cos(t4)*np.sin(t1)+ (538*np.cos(t2)*np.sin(t1)*np.sin(t3)*np.sin(t4)/5 + (538*np.cos(t3)*np.sin(t1)*np.sin(t2)*np.sin(t4)/5 + (538*np.cos(t4)*np.sin(t1)*np.sin(t2)*np.sin(t3)/5 
J12 = 100*np.cos(t1)*np.cos(t2) - 35*np.cos(t1)*np.sin(t2)+ 100*np.cos(t1)*np.cos(t2)*np.sin(t3) - 100*np.cos(t1)*np.cos(t3)*np.sin(t2)- (538*np.cos(t1)*np.cos(t2)*np.cos(t3)*np.sin(t4)/5 - (538*np.cos(t1)*np.cos(t2)*np.cos(t4)*np.sin(t3)/5 - (538*np.cos(t1)*np.cos(t3)*np.cos(t4)*np.sin(t2)/5 - (538*np.cos(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J13 = 100*np.cos(t1)*np.cos(t2)*np.sin(t3) - 100*np.cos(t1)*np.cos(t3)*np.sin(t2)- (538*np.cos(t1)*np.cos(t2)*np.cos(t3)*np.sin(t4)*/5 - (538*np.cos(t1)*np.cos(t2)*np.cos(t4)*np.sin(t3)/5 - (538*np.cos(t1)*np.cos(t3)*np.cos(t4)*np.sin(t2)/5+ (538*np.cos(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J14 = (538*np.cos(t1)*np.cos(t2)*np.cos(t3)*np.sin(t4)/5 - (538*np.cos(t1)*np.cos(t2)*np.cos(t4)*np.sin(t3)/5 - (538*np.cos(t1)*np.cos(t3)*np.cos(t4)*np.sin(t2/5 + (538*np.cos(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J21 = 35*np.cos(t1)*np.cos(t2)+ 100*np.cos(t1)*np.sin(t2) + 100*np.cos(t1)*np.cos(t2)*np.cos(t3)- 100*np.cos(t1)*np.sin(t2)*np.sin(t3) + (538*np.cos(t1)*np.cos(t2)*np.cos(t3)*np.cos(t4)- (538*np.cos(t1)*np.cos(t2)*np.sin(t3)*np.sin(t4)*diff(t1, t))/5 - (538*np.cos(t1)*np.cos(t3)*np.sin(t2)*np.sin(t4)*diff(t1, t))/5 - (538*np.cos(t1)*np.cos(t4)*np.sin(t2)*np.sin(t3)*diff(t1, t))/5 
J22 =100*np.cos(t2)*np.sin(t1) - 35*np.sin(t1)*np.sin(t2) - 100*np.cos(t2)*np.sin(t1)*np.sin(t3)- 100*np.cos(t3)*np.sin(t1)*np.sin(t2) - (538*np.cos(t2)*np.cos(t3)*np.sin(t1)*np.sin(t4)/5 - (538*np.cos(t2)*np.cos(t4)*np.sin(t1)*np.sin(t3)/5 - (538*np.cos(t3)*np.cos(t4)*np.sin(t1)*np.sin(t2)/5+ (538*np.sin(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J23 = - 100*np.cos(t2)*np.sin(t1)*np.sin(t3)- 100*np.cos(t3)*np.sin(t1)*np.sin(t2) - (538*np.cos(t2)*np.cos(t3)*np.sin(t1)*np.sin(t4)/5 - (538*np.cos(t2)*np.cos(t4)*np.sin(t1)*np.sin(t3)/5 - (538*np.cos(t3)*np.cos(t4)*np.sin(t1)*np.sin(t2)/5 + (538*np.sin(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J24 = - (538*np.cos(t2)*np.cos(t3)*np.sin(t1)*np.sin(t4)/5 - (538*np.cos(t2)*np.cos(t4)*np.sin(t1)*np.sin(t3)/5 - (538*np.cos(t3)*np.cos(t4)*np.sin(t1)*np.sin(t2)/5 + (538*np.sin(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J31 = 35*np.cos(t1)*np.cos(t2) + 100*np.cos(t1)*np.sin(t2) + 100*np.cos(t1)*np.cos(t2)*np.cos(t3)- 100*np.cos(t1)*np.sin(t2)*np.sin(t3) + (538*np.cos(t1)*np.cos(t2)*np.cos(t3)*np.cos(t4) 
J32 =100*np.cos(t2)*np.sin(t1) - 35*np.sin(t1)*np.sin(t2) - 100*np.cos(t2)*np.sin(t1)*np.sin(t3) - 100*np.cos(t3)*np.sin(t1)*np.sin(t2) - (538*np.cos(t2)*np.cos(t3)*np.sin(t1)*np.sin(t4)/5 - (538*np.cos(t2)*np.cos(t4)*np.sin(t1)*np.sin(t3)/5 - (538*np.cos(t3)*np.cos(t4)*np.sin(t1)*np.sin(t2)/5 + (538*np.sin(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J33 = - 100*np.cos(t2)*np.sin(t1)*np.sin(t3) - 100*np.cos(t3)*np.sin(t1)*np.sin(t2) - (538*np.cos(t2)*np.cos(t3)*np.sin(t1)*np.sin(t4)/5 - (538*np.cos(t2)*np.cos(t4)*np.sin(t1)*np.sin(t3)/5 - (538*np.cos(t3)*np.cos(t4)*np.sin(t1)*np.sin(t2)/5 + (538*np.sin(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
J34 = - (538*np.cos(t2)*np.cos(t3)*np.sin(t1)*np.sin(t4)/5 - (538*np.cos(t2)*np.cos(t4)*np.sin(t1)*np.sin(t3)/5 - (538*np.cos(t3)*np.cos(t4)*np.sin(t1)*np.sin(t2)/5 + (538*np.sin(t1)*np.sin(t2)*np.sin(t3)*np.sin(t4)/5 
                                                                                                                                                                        
# Compute twist using jacobian
    self.J = np.array([[J11,J12,J13,J14],[J21,J22,J23,J24],[J31,J32,J33,J34]])