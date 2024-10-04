% Robot_Model.m
% Descrption: A function that creates the robot model according to the DH parameters 
        % and shows it with offset thetas.
% Input: DH (a struct containing d, a, alpha, offset)
% Output: robot(a Serial-link robot class)

function robot = Robot_Model(DH)
%% Create the robot model
L(1) = Link('revolute', 'd', DH.d(1), 'a', DH.a(1), 'alpha', DH.alpha(1));
L(2) = Link('revolute', 'd', DH.d(2), 'a', DH.a(2), 'alpha', DH.alpha(2));
L(3) = Link('revolute', 'd', DH.d(3), 'a', DH.a(3), 'alpha', DH.alpha(3));
L(4) = Link('revolute', 'd', DH.d(4), 'a', DH.a(4), 'alpha', DH.alpha(4));
L(5) = Link('revolute', 'd', DH.d(5), 'a', DH.a(5), 'alpha', DH.alpha(5));
L(6) = Link('revolute', 'd', DH.d(6), 'a', DH.a(6), 'alpha', DH.alpha(6));
robot = SerialLink(L, 'name', 'robot');

%% Set joint variable limits (according to JAKA Zu 7)
degree=pi/180;
L(1).qlim =[-360*degree, 360*degree];
L(2).qlim =[-85*degree, 265*degree];    % if with offset: [-175*degree, 175*degree]
L(3).qlim =[-175*degree, 175*degree];
L(4).qlim =[-85*degree, 265*degree];    % if with offset: [-175*degree, 175*degree]
L(5).qlim =[-360*degree, 360*degree];
L(6).qlim =[-360*degree, 360*degree]; 

%% Show the robot model
robot.plot([DH.offset(1) DH.offset(2) DH.offset(3) DH.offset(4) DH.offset(5) DH.offset(6)]); 
% the offset theta  % 为了后面运动学正反解的比较更加方便，并未将其设置为Link内置的offset。                                                                              

end
