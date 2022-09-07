function [q_eu,q_ed] = ikinUR3e(X)

L = [151.8440 243.5180 212.8800 132.0337 85.2541 92.368];

a1 = -X(2);
b1 = -X(1);
c1 = sqrt((a1^2)+(b1^2));
alpha1 = atan2(a1,b1);
a2 = L(4);
b2 = sqrt((c1^2)+(a2^2));
alpha2 = atan2(a2,b2);
theta1 = alpha1 - alpha2;
c3 = b2-L(5);
a4 = (X(3)+L(6))-L(1);
c4 = sqrt((c3^2)+(a4^2));
alpha4 = atan2(a4,c3);
b5 = L(2);
a5 = L(3);
alpha5 = acos(((b5^2)+(c4^2)-(a5^2))/(2*b5*c4));
gamma5 = acos(((a5^2)+(b5^2)-(c4^2))/(2*a5*b5));
theta2_eu = -(alpha4+alpha5);
theta3_eu = pi-gamma5;
theta2_ed = -(alpha4-alpha5);
theta3_ed = gamma5-pi;
theta4_eu = -(theta2_eu+theta3_eu+(pi/2));
theta4_ed = -(theta2_ed+theta3_ed+(pi/2));
theta5 = pi/2;
theta6 = (pi/2) + theta1 - pi/4;  % angle of end effector
q_eu = [theta1 theta2_eu theta3_eu theta4_eu theta5 theta6];
q_ed = [theta1 theta2_ed theta3_ed theta4_ed theta5 theta6];

clear L

end