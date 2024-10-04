
% All_kine.m
% Descrption: Gathering and showing all fundamental kinematics functions here, 
            % including the programming of forward kinematics, inverse kinematics and Jacobian matrix.
% Custom Functions:
    % [robot] = Robot_Model(DH)     : the function creating the robot model and showing it with offset thetas
    % [T] = fkine_c(q,DH)           : the custom forward kinematics function
    % [q_all] = ikine_c_all(T,DH)   : the custom inverse kinematics function showing all 8 analytical solutions
    % [q] = ikine_numerical(T,DH)   : the custom inverse kinematics function showing the numerical solution
    % [J] = jacob(q,DH)             : the custom function calculating Jacobian matrix in world frame
    % [J_T] = jacob_T(q,DH)         : the custom function calculating Jacobian matrix in tool frame


%% DH Parameters(简化版)

DH.d = [120.15 144.15 0 -29.14 113.5 107];      % d_i
DH.a = [0 350 294.5 0 0 0];                     % a_i
DH.alpha = [pi/2 0 0 -pi/2 pi/2 0];             % alpha_i
DH.offset = [0 pi/2 0 -pi/2 0 0];

%% Create the robot model

robot = Robot_Model(DH);

%% Forward Kinematics

q_f = [pi/3 pi/2 pi/3 pi/2 pi/6 pi/6];  % without offset
T_f = robot.fkine(q_f);
T_f_c = fkine_c(q_f,DH);
disp('----------------------Forward Kinematics-------------------------');
disp('The result of the forward kinematics function in Robtotics Toolbox:');
disp(T_f);
disp('The result of the custom forward kinematics function:');
disp(T_f_c);

%% Inverse Kinematics: All Solutions

T_i = [-0.6725    0.5377    0.5085     128.7;
       -0.2988    0.4313   -0.8513    -192.5;
       -0.6771   -0.7244   -0.1294       447;
             0         0         0         1];
q_i = robot.ikine(T_i);
q_i_all = ikine_c_all(T_i,DH);
disp('----------------------Inverse Kinematics-------------------------');
disp('The solution of the inverse kinematics function in Robtotics Toolbox:');
disp(q_i);
disp('The 8 analytical solutions of the custom inverse kinematics function:');
disp(['solution 1: ',mat2str(q_i_all(1,:))]);
disp(['solution 2: ',mat2str(q_i_all(2,:))]);
disp(['solution 3: ',mat2str(q_i_all(3,:))]);
disp(['solution 4: ',mat2str(q_i_all(4,:))]);
disp(['solution 5: ',mat2str(q_i_all(5,:))]);
disp(['solution 6: ',mat2str(q_i_all(6,:))]);
disp(['solution 7: ',mat2str(q_i_all(7,:))]);
disp(['solution 8: ',mat2str(q_i_all(8,:))]);
disp('The numerical solution of the custom inverse kinematics function:');
q_i_num = ikine_numerical(T_i,DH);

%% Jacobian Matrix
q_J = [pi/6 pi/3 pi/2 pi/6 pi/6 pi/3];
% In World Frame
J = robot.jacob0(q_J);
J_c = jacob(q_J,DH);
disp('----------------------Jacobian Matrix-------------------------');
disp('The Jacobian matrix in world frame calculating by the `jacob0` function in Robtotics Toolbox:');
disp(J);
disp('The Jacobian matrix in world frame calculating by the custom function:');
disp(J_c);
% In Tool Frame
J_T = robot.jacobn(q_J);
J_T_c = jacob_T(q_J,DH);
disp('The Jacobian matrix in tool frame calculating by the `jacobn` function in Robtotics Toolbox:');
disp(J_T);
disp('The Jacobian matrix in tool frame calculating by the custom function:');
disp(J_T_c);
