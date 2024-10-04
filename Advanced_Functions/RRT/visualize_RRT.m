% visualize_RRT.m
% Descrption: Visualize the RRT result, extract the optimal path and smooth it.
% Output: 'robot_movie.mp4'

%% Create the Space with obstacles

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

%T = [0 0 1 300; 0 -1 0 -500; 1 0 0 100; 0 0 0 1];
%q = robot.ikine(T);
%robot.plot(q); 
T_C1 = [1 1 1];     % 'tile1color', C	 Color of even tiles [r g b] (default [0.5 1 0.5] light green)
T_C2 = [1 1 1];  % 'tile2color', C	 Color of odd tiles [r g b] (default [1 1 1] white)
robot.plot(DH.offset,'tile1color', T_C1,'tile2color', T_C2);
hold on;


%% All Results of RRT nodes

data = readmatrix('RRT.xlsx');
for i = 2:size(data,1)
    parent_node = data(i,8);
    T_parent = robot.fkine([data(parent_node,1) data(parent_node,2) data(parent_node,3) data(parent_node,4) data(parent_node,5) data(parent_node,6)]).T;
    T_i = robot.fkine([data(i,1) data(i,2) data(i,3) data(i,4) data(i,5) data(i,6)]).T;
    plot3([T_parent(1,4),T_i(1,4)],[T_parent(2,4),T_i(2,4)],[T_parent(3,4),T_i(3,4)],'LineWidth',2,'Color',"#E3CF57");
    hold on;
end

%% Find the optimal result of RRT

node = 772; % from the final step
optimal_path = data(node,1:6);
for j = 1:300
    node = data(node,8);
    optimal_path = cat(1,data(node,1:6),optimal_path);
    if node == 1
        break;
    end
end

for m = 2:size(optimal_path,1)
    T_before = robot.fkine([optimal_path(m-1,1) optimal_path(m-1,2) optimal_path(m-1,3) optimal_path(m-1,4) optimal_path(m-1,5) optimal_path(m-1,6)]).T;
    T_after = robot.fkine([optimal_path(m,1) optimal_path(m,2) optimal_path(m,3) optimal_path(m,4) optimal_path(m,5) optimal_path(m,6)]).T;
    plot3([T_before(1,4),T_after(1,4)],[T_before(2,4),T_after(2,4)],[T_before(3,4),T_after(3,4)],'LineWidth',4,'Color',"#ED9121");
    hold on;
end

%% Smooth the trajectory using 'jtraj'

[q_all,qd_all,qdd_all] = jtraj(optimal_path(1,:), optimal_path(2,:), 5,[0 0 0 0 0 0],(optimal_path(2,:)-optimal_path(1,:))/2); % 对于开始点，起始速度为0
for n = 3:(size(optimal_path,1)-1)
    qd_mean_1 = (optimal_path(n,:)-optimal_path(n-2,:))/2;      % 第n-1个轨迹点的平均速度，作为这次插值的起始速度
    qd_mean_2 = (optimal_path(n+1,:)-optimal_path(n-1,:))/2;     % 第n个轨迹点的平均速度，作为这次插值的终止速度
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
robot.plot(q_all,'trail', 'r--', 'movie', 'robot_movie.mp4');
