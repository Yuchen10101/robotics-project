% fkine_c.m
% Descrption: A custom forward kinematics function
% Input: DH (a struct containing d, a, alpha, offset); q (1xN joint configuration)
% Output: T (the pose of the robot end-effector, as an 4x4 SE(3) homogeneous transformation)

function T = fkine_c(q,DH)

%% Elements of T

c1 = cos(q(1));     s1 = sin(q(1));     d1 = DH.d(1);
c2 = cos(q(2));     s2 = sin(q(2));     a2 = DH.a(2);       d2 = DH.d(2);
c23 = cos(q(2)+q(3));       s23 = sin(q(2)+q(3));       a3 = DH.a(3);   
c234 = cos(q(2)+q(3)+q(4));     s234 = sin(q(2)+q(3)+q(4));     d4 = DH.d(4);
c5 = cos(q(5));     s5 = sin(q(5));     d5 = DH.d(5);
c6 = cos(q(6));     s6 = sin(q(6));     d6 = DH.d(6);

nx = - c1*s6*s234 - s1*s5*c6 + c1*c5*c6*c234;
ny = c1*s6*c6 + s1*c5*c6*c234 - s1*s6*s234;
nz = s6*c234 + c5*c6*s234;
ox = s1*s5*s6 - c1*c5*s6*c234 - c1*c6*s234;
oy = - s1*c6*s234 - c1*s5*s6 - s1*c5*s6*c234;
oz = c6*c234 - c5*s6*s234;
ax = s1*c5 + c1*s5*c234;
ay = s1*s5*c234 - c1*c5;
az = s5*s234;
px = (d2+d4)*s1 - d5*c1*s234 + d6*s1*c5 + d6*c1*s5*c234 + a2*c1*c2 + a3*c1*c23;
py = -(d2+d4)*c1 - d5*s1*s234 - d6*c1*c5 + d6*s1*s5*c234 + a2*s1*c2 + a3*s1*c23;
pz = d1 + a2*s2 + a3*s23 + d5*c234 + d6*s5*s234;

%% T
T = [nx ox ax px; ny oy ay py; nz oz az pz; 0 0 0 1];

end
