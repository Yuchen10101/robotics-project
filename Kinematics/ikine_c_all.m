% ikine_c.m
% Descrption: A custom inverse kinematics function
% Input: DH (a struct containing d, a, alpha, offset); 
       % T (the pose of the end-effector as an 4x4 SE(3) homogeneous transformation)
% Output: q_all (all 8 solutions of joint configuration)

function q_all = ikine_c_all(T,DH)

%% Rename the elements of T and DH
% T
nx = T(1,1);   ox = T(1,2);   ax = T(1,3);   px = T(1,4);
ny = T(2,1);   oy = T(2,2);   ay = T(2,3);   py = T(2,4);
nz = T(3,1);   oz = T(3,2);   az = T(3,3);   pz = T(3,4);

% DH
d1 = DH.d(1);   d2 = DH.d(2);   d4 = DH.d(4);   d5 = DH.d(5);   d6 = DH.d(6);
a2 = DH.a(2);   a3 = DH.a(3);


%% theta 1

theta1_1 = wrapToPi(atan2(py-d6*ay,px-d6*ax)+atan2(d2+d4,sqrt((px-d6*ax)^2+(py-d6*ay)^2-(d2+d4)^2)));
theta1_2 = wrapToPi(atan2(py-d6*ay,px-d6*ax)+atan2(d2+d4,-sqrt((px-d6*ax)^2+(py-d6*ay)^2-(d2+d4)^2)));
theta1 = [theta1_1 theta1_1 theta1_1 theta1_1 theta1_2 theta1_2 theta1_2 theta1_2];


%% theta 5

% theta1_1 -> theta5_1 theta5_2
theta5_1 = wrapToPi(acos(ax*sin(theta1_1) - ay*cos(theta1_1)));
theta5_2 = wrapToPi(-acos(ax*sin(theta1_1) - ay*cos(theta1_1)));

% theta1_2 -> theta5_3 theta5_4
theta5_3 = wrapToPi(acos(ax*sin(theta1_2) - ay*cos(theta1_2)));
theta5_4 = wrapToPi(-acos(ax*sin(theta1_2) - ay*cos(theta1_2)));

theta5 = [theta5_1 theta5_1 theta5_2 theta5_2 theta5_3 theta5_3 theta5_4 theta5_4];


%% theta 6

% theta1_1 theta5_1 -> theta6_1
theta6_1 = wrapToPi(atan2((ox*sin(theta1_1)-oy*cos(theta1_1))/sin(theta5_1),(-nx*sin(theta1_1)+ny*cos(theta1_1))/sin(theta5_1)));

% theta1_1 theta5_2 -> theta6_2
theta6_2 = wrapToPi(atan2((ox*sin(theta1_1)-oy*cos(theta1_1))/sin(theta5_2),(-nx*sin(theta1_1)+ny*cos(theta1_1))/sin(theta5_2)));

% theta1_2 theta5_3 -> theta6_3
theta6_3 = wrapToPi(atan2((ox*sin(theta1_2)-oy*cos(theta1_2))/sin(theta5_3),(-nx*sin(theta1_2)+ny*cos(theta1_2))/sin(theta5_3)));

% theta1_2 theta5_4 -> theta6_4
theta6_4 = wrapToPi(atan2((ox*sin(theta1_2)-oy*cos(theta1_2))/sin(theta5_4),(-nx*sin(theta1_2)+ny*cos(theta1_2))/sin(theta5_4)));

theta6 = [theta6_1 theta6_1 theta6_2 theta6_2 theta6_3 theta6_3 theta6_4 theta6_4];


%% theta 2+3+4

% theta1_1 theta5_1 -> theta234_1
theta234_1 = wrapToPi(atan2((az)/sin(theta5_1),(ax*cos(theta1_1)+ay*sin(theta1_1))/sin(theta5_1)));

% theta1_1 theta5_2 -> theta234_2
theta234_2 = wrapToPi(atan2((az)/sin(theta5_2),(ax*cos(theta1_1)+ay*sin(theta1_1))/sin(theta5_2)));

% theta1_2 theta5_3 -> theta234_3
theta234_3 = wrapToPi(atan2((az)/sin(theta5_3),(ax*cos(theta1_2)+ay*sin(theta1_2))/sin(theta5_3)));

% theta1_2 theta5_4 -> theta234_4
theta234_4 = wrapToPi(atan2((az)/sin(theta5_4),(ax*cos(theta1_2)+ay*sin(theta1_2))/sin(theta5_4)));

theta234 = [theta234_1 theta234_1 theta234_2 theta234_2 theta234_3 theta234_3 theta234_4 theta234_4];


%% theta 2, 3, 4

% initialize
theta23 = [0 0 0 0 0 0 0 0];
theta2 = [0 0 0 0 0 0 0 0];
theta3 = [0 0 0 0 0 0 0 0];
theta4 = [0 0 0 0 0 0 0 0];

K1 = px*cos(theta1)+py*sin(theta1)+d5*sin(theta234)-d6*sin(theta5).*cos(theta234);
K2 = pz-d1-d5*cos(theta234)-d6*sin(theta5).*sin(theta234);

% theta1 theta5 theta234 -> theta23 -> theta3&theta4 -> theta2
for i = 1:4
    k1 = K1(2*i - 1);
    k2 = K2(2*i - 1);
    theta23(2*i - 1) = atan2(k1^2+k2^2+a3^2-a2^2,sqrt(4*a3^2*(k1^2+k2^2)-(k1^2+k2^2+a3^2-a2^2)^2)) - atan2(k1,k2);
    theta4(2*i - 1) = theta234(2*i - 1) - theta23(2*i - 1);
    theta3(2*i - 1) = atan2(-k2*cos(theta23(2*i - 1))+k1*sin(theta23(2*i - 1)),k1*cos(theta23(2*i - 1))+k2*sin(theta23(2*i - 1))-a3);
    theta2(2*i - 1) = theta23(2*i - 1) - theta3(2*i - 1);
    k1 = K1(2*i);
    k2 = K2(2*i);
    theta23(2*i) = atan2(k1^2+k2^2+a3^2-a2^2,-sqrt(4*a3^2*(k1^2+k2^2)-(k1^2+k2^2+a3^2-a2^2)^2))-atan2(k1,k2);
    theta4(2*i) = theta234(2*i) - theta23(2*i);
    theta3(2*i) = atan2(-k2*cos(theta23(2*i))+k1*sin(theta23(2*i)),k1*cos(theta23(2*i))+k2*sin(theta23(2*i))-a3);
    theta2(2*i) = theta23(2*i) - theta3(2*i);
end

theta2 = wrapToPi(theta2);
theta3 = wrapToPi(theta3);
theta4 = wrapToPi(theta4);


%% Arrange all solutions
q_all = zeros(8,6);
for i = 1:8
    q_all(i,:) = [theta1(i) theta2(i) theta3(i) theta4(i) theta5(i) theta6(i)];
end
q_all = round(q_all,4);

end
