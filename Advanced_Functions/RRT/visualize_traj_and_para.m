% visualize_traj_and_para.m
% Descrption: Visualize the final trajectory, show some parameters(q qd qdd) of all joints.
% Output: 1) 'robo_movie_only_traj.mp4' 
        % 2) fig showing q & qd & qdd of all joints

[X,Y,Z] = sphere;
r = [200 150 300 50];    % Radius
O = [0 -600 800; 450 -100 500; 0 600 300; 300 -500 100];     % Origin
X1 = X * r(1);  Y1 = Y * r(1);  Z1 = Z * r(1);
X2 = X * r(2);  Y2 = Y * r(2);  Z2 = Z * r(2);
X3 = X * r(3);  Y3 = Y * r(3);  Z3 = Z * r(3);
X4 = X * r(4);  Y4 = Y * r(4);  Z4 = Z * r(4);
surf(X1+O(1,1),Y1+O(1,2),Z1+O(1,3));
hold on;
surf(X2+O(2,1),Y2+O(2,2),Z2+O(2,3));
hold on;
surf(X3+O(3,1),Y3+O(3,2),Z3+O(3,3));
hold on;
surf(X4+O(4,1),Y4+O(4,2),Z4+O(4,3));
hold on;

%% Create the robot model

DH.d = [120.15 144.15 -142.64 113.5 113.5 107];   % d_i
DH.a = [0 350 294.5 0 0 0];                       % a_i
DH.alpha = [pi/2 0 0 -pi/2 pi/2 0];               % alpha_i
DH.offset = [0 pi/2 0 -pi/2 0 0];

L(1) = Link('revolute', 'd', DH.d(1), 'a', DH.a(1), 'alpha', DH.alpha(1));
L(2) = Link('revolute', 'd', DH.d(2), 'a', DH.a(2), 'alpha', DH.alpha(2));
L(3) = Link('revolute', 'd', DH.d(3), 'a', DH.a(3), 'alpha', DH.alpha(3));
L(4) = Link('revolute', 'd', DH.d(4), 'a', DH.a(4), 'alpha', DH.alpha(4));
L(5) = Link('revolute', 'd', DH.d(5), 'a', DH.a(5), 'alpha', DH.alpha(5));
L(6) = Link('revolute', 'd', DH.d(6), 'a', DH.a(6), 'alpha', DH.alpha(6));
robot = SerialLink(L, 'name', 'robot');

degree=pi/180;
L(1).qlim =[-360*degree, 360*degree];
L(2).qlim =[-85*degree, 265*degree];    % if with offset: [-175*degree, 175*degree]
L(3).qlim =[-175*degree, 175*degree];
L(4).qlim =[-85*degree, 265*degree];    % if with offset: [-175*degree, 175*degree]
L(5).qlim =[-360*degree, 360*degree];
L(6).qlim =[-360*degree, 360*degree]; 

T_C1 = [1 1 1];     % 'tile1color', C	 Color of even tiles [r g b] (default [0.5 1 0.5] light green)
T_C2 = [1 1 1];  % 'tile2color', C	 Color of odd tiles [r g b] (default [1 1 1] white)
robot.plot(DH.offset,'tile1color', T_C1,'tile2color', T_C2);
hold on;


%% Optimal Path
data = readmatrix('RRT.xlsx');
node = 772; % from the final step
optimal_path = data(node,1:6);
for j = 1:200
    node = data(node,8);
    optimal_path = cat(1,data(node,1:6),optimal_path);
    if node == 1
        break;
    end
end

%% Show only the trajectory 

[q_all,qd_all,qdd_all] = jtraj(optimal_path(1,:), optimal_path(2,:), 5,[0 0 0 0 0 0],(optimal_path(2,:)-optimal_path(1,:))/2);
for n = 3:(size(optimal_path,1)-1)
    qd_mean_1 = (optimal_path(n,:)-optimal_path(n-2,:))/2; 
    qd_mean_2 = (optimal_path(n+1,:)-optimal_path(n-1,:))/2;
    [q,qd,qdd] = jtraj(optimal_path(n-1,:), optimal_path(n,:),5,qd_mean_1,qd_mean_2);
    q_all = cat(1,q_all,q(2:5,:));
    qd_all = cat(1,qd_all,qd(2:5,:));
    qdd_all = cat(1,qdd_all,qdd(2:5,:));
end
[qf,qdf,qddf] = jtraj(optimal_path(size(optimal_path,1)-1,:), optimal_path(size(optimal_path,1),:),...
    15,(optimal_path(size(optimal_path,1),:)-optimal_path(size(optimal_path,1)-2,:))/2,[0 0 0 0 0 0]);   % 对于结束点，终止速度为0
q_all = cat(1,q_all,qf(2:15,:));
qd_all = cat(1,qd_all,qdf(2:15,:));
qdd_all = cat(1,qdd_all,qddf(2:15,:));
robot.plot(q_all,'trail', 'r--', 'movie', 'robo_movie_only_traj.mp4');


%% Showing the variation of some parameters during trajectory
% theta 1
q_theta1 = q_all(:,1);
qd_theta1 = qd_all(:,1);
qdd_theta1 = qdd_all(:,1);
% theta 2
q_theta2 = q_all(:,2);
qd_theta2 = qd_all(:,2);
qdd_theta2 = qdd_all(:,2);
% theta 3
q_theta3 = q_all(:,3);
qd_theta3 = qd_all(:,3);
qdd_theta3 = qdd_all(:,3);
% theta 4
q_theta4 = q_all(:,4);
qd_theta4 = qd_all(:,4);
qdd_theta4 = qdd_all(:,4);
% theta 5
q_theta5 = q_all(:,5);
qd_theta5 = qd_all(:,5);
qdd_theta5 = qdd_all(:,5);
% theta 6
q_theta6 = q_all(:,6);
qd_theta6 = qd_all(:,6);
qdd_theta6 = qdd_all(:,6);

figure(2)
% theta1
subplot(6,3,1);
plot(q_theta1,'linewidth',1.5,'Color',[0.53 0.15 0.34]);
legend('q:theta 1');
subplot(6,3,2);
plot(qd_theta1,'linewidth',1.5,'Color',[0.53 0.15 0.34]);
legend('qd:theta 1');
subplot(6,3,3);
plot(qdd_theta1,'linewidth',1.5,'Color',[0.53 0.15 0.34]);
legend('qdd:theta 1');
% theta2
subplot(6,3,4);
plot(q_theta2,'linewidth',1.5,'Color',[0.89 0.81 0.34]);
legend('q:theta 2');
subplot(6,3,5);
plot(qd_theta2,'linewidth',1.5,'Color',[0.89 0.81 0.34]);
legend('qd:theta 2');
subplot(6,3,6);
plot(qdd_theta2,'linewidth',1.5,'Color',[0.89 0.81 0.34]);
legend('qdd:theta 2');
% theta3
subplot(6,3,7);
plot(q_theta3,'linewidth',1.5,'Color',[0.74 0.56 0.56]);
legend('q:theta 3');
subplot(6,3,8);
plot(qd_theta3,'linewidth',1.5,'Color',[0.74 0.56 0.56]);
legend('qd:theta 3');
subplot(6,3,9);
plot(qdd_theta3,'linewidth',1.5,'Color',[0.74 0.56 0.56]);
legend('qdd:theta 3');
% theta4
subplot(6,3,10);
plot(q_theta4,'linewidth',1.5,'Color',[0.24 0.35 0.67]);
legend('q:theta 4');
subplot(6,3,11);
plot(qd_theta4,'linewidth',1.5,'Color',[0.24 0.35 0.67]);
legend('qd:theta 4');
subplot(6,3,12);
plot(qdd_theta4,'linewidth',1.5,'Color',[0.24 0.35 0.67]);
legend('qdd:theta 4');
% theta5
subplot(6,3,13);
plot(q_theta5,'linewidth',1.5,'Color',[0.01 0.66 0.64]);
legend('q:theta 5');
subplot(6,3,14);
plot(qd_theta5,'linewidth',1.5,'Color',[0.01 0.66 0.64]);
legend('qd:theta 5');
subplot(6,3,15);
plot(qdd_theta5,'linewidth',1.5,'Color',[0.01 0.66 0.64]);
legend('qdd:theta 5');
% theta6
subplot(6,3,16);
plot(q_theta6,'linewidth',1.5,'Color',[0.22 0.37 0.06]);
legend('q:theta 6');
subplot(6,3,17);
plot(qd_theta6,'linewidth',1.5,'Color',[0.22 0.37 0.06]);
legend('qd:theta 6');
subplot(6,3,18);
plot(qdd_theta6,'linewidth',1.5,'Color',[0.22 0.37 0.06]);
legend('qdd:theta 6');
