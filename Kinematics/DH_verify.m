
%% 原始DH参数

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
robot1 = SerialLink(L, 'name', 'robot1');


%% 简化DH参数

dH.d = [120.15 144.15 0 -29.14 113.5 107];      % d_i
dH.a = [0 350 294.5 0 0 0];                     % a_i
dH.alpha = [pi/2 0 0 -pi/2 pi/2 0];             % alpha_i
dH.offset = [0 pi/2 0 -pi/2 0 0];

l(1) = Link('revolute', 'd', dH.d(1), 'a', dH.a(1), 'alpha', dH.alpha(1));
l(2) = Link('revolute', 'd', dH.d(2), 'a', dH.a(2), 'alpha', dH.alpha(2));
l(3) = Link('revolute', 'd', dH.d(3), 'a', dH.a(3), 'alpha', dH.alpha(3));
l(4) = Link('revolute', 'd', dH.d(4), 'a', dH.a(4), 'alpha', dH.alpha(4));
l(5) = Link('revolute', 'd', dH.d(5), 'a', dH.a(5), 'alpha', dH.alpha(5));
l(6) = Link('revolute', 'd', dH.d(6), 'a', dH.a(6), 'alpha', dH.alpha(6));
robot2 = SerialLink(l, 'name', 'robot2');


%% 验证两者等效
%%%% 正向运动学
q_f = [pi/3 pi/2 pi/3 pi/2 pi/6 pi/6];  % without offset
T_f_1 = robot1.fkine(q_f);
T_f_2 = robot2.fkine(q_f);
disp('正向运动学:');
disp('由原始DH参数计算的T:');
disp(T_f_1);
disp('由简化DH参数计算的T:');
disp(T_f_2);
%%%% 逆向运动学
T_i = [ -0.8256    0.4641    0.3209     172.6;
        -0.5640   -0.6962   -0.4441     182.4;
         0.0173   -0.5477    0.8365     645.2;
              0         0         0         1];
q_i_1 = robot1.ikine(T_i);
q_i_2 = robot2.ikine(T_i);
disp('逆向运动学:');
disp('由原始DH参数计算的q:');
disp(q_i_1);
disp('由简化DH参数计算的q:');
disp(q_i_2);
