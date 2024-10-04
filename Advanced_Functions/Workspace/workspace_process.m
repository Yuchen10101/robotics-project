% workspace_process.m
% Descrption: showing the process of sampling points in workspace

%% DH Parameters

DH.d = [120.15 144.15 -142.64 113.5 113.5 107];   % d_i
DH.a = [0 350 294.5 0 0 0];                       % a_i
DH.alpha = [pi/2 0 0 -pi/2 pi/2 0];               % alpha_i
DH.offset = [0 pi/2 0 -pi/2 0 0];

%% Create the robot model

L(1) = Link('revolute', 'd', DH.d(1), 'a', DH.a(1), 'alpha', DH.alpha(1));
L(2) = Link('revolute', 'd', DH.d(2), 'a', DH.a(2), 'alpha', DH.alpha(2));
L(3) = Link('revolute', 'd', DH.d(3), 'a', DH.a(3), 'alpha', DH.alpha(3));
L(4) = Link('revolute', 'd', DH.d(4), 'a', DH.a(4), 'alpha', DH.alpha(4));
L(5) = Link('revolute', 'd', DH.d(5), 'a', DH.a(5), 'alpha', DH.alpha(5));
L(6) = Link('revolute', 'd', DH.d(6), 'a', DH.a(6), 'alpha', DH.alpha(6));
robot = SerialLink(L, 'name', 'robot');

%% Set joint variable limits and sample the points

teach(robot); 
robot.plot([DH.offset(1) DH.offset(2) DH.offset(3) DH.offset(4) DH.offset(5) DH.offset(6)])
hold on;
N=30000;

limitmax_1 = -360;
limitmin_1 = 360;
limitmax_2 = -85;
limitmin_2 = 265;
limitmax_3 = -175;
limitmin_3 = 175;
limitmax_4 = -85;
limitmin_4 = 265;
limitmax_5 = -360;
limitmin_5 = 360;
limitmax_6 = -360;
limitmin_6 = 360;

theta1=(limitmin_1+(limitmax_1-limitmin_1)*rand(N,1))*pi/180; 
theta2=(limitmin_2+(limitmax_2-limitmin_2)*rand(N,1))*pi/180; 
theta3=(limitmin_3+(limitmax_3-limitmin_3)*rand(N,1))*pi/180; 
theta4=(limitmin_4+(limitmax_4-limitmin_4)*rand(N,1))*pi/180;
theta5=(limitmin_5+(limitmax_5-limitmin_5)*rand(N,1))*pi/180;
theta6=(limitmin_6+(limitmax_6-limitmin_6)*rand(N,1))*pi/180;

%% Showing Progress

for n=1:1:3000
qq=[theta1(n),theta2(n),theta3(n),theta4(n),theta5(n),theta6(n)];
robot.plot(qq);
Mricx=robot.fkine(qq);
plot3(Mricx.t(1),Mricx.t(2),Mricx.t(3),'b.','MarkerSize',0.5);
hold on;
end