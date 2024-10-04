% RRT_traj.m
% Descrption: Trajectory planning using RRT(Rapidly Exploring Random Tree) method to avoid obstacles.

% Functions: 
        % 1. RRT method: 1.1 <- 1.2
            % 1.1 main process:
            %   [nodeList,isFind] = RRT(robot,initJointPos,ballsOrigin,ballsRadius,cylindersInfo,index): main RRT process
            % 1.2 Generate new node:
            %   [newNode] = SampleNearAndSteer(nodeList,angleStepSize,initJointPos,targetJointPos)
        % 2. Collison Check: 2.1 <- 2.2 <- 2.3
            % 2.1 Check path collison between newNode and nearNode:
            %   [noCollision] = CheckPathCollision(newNode,nearNode,threhold,ballsOrigin,ballsRadius,cylindersInfo,index)
            % 2.2 Check links collision with balls
            %   [noCollision] = CheckLinksCollision(q,threhold,ballsOrigin,ballsRadius,cylindersInfo,index)
            % 2.3 Check single cylinder collision with single ball
            %   [noCollision] = CheckSingleLinkCollision(idx,cylinderInfo,threhold,origin,ballRadius)
        % 3.Other functions:
            % 3.1 Forward kinemics of all joints:
            %   [T01,T02,T03,T04,T05,T06] = RobotFkine(q)

% Output: RRT.xlsx (col1-6 :jointpos, col7:cost, col8:parent node)
         % If there is already a file named 'RRT.xlsx', remove it cause 'writematrix' function will write after the initial content.

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

%% Main Process

initJointPos = DH.offset;
ballsOrigin = [0 -600 800; 450 -100 500; 0 600 300; 300 -500 100];
ballsRadius = [200 150 300 50];
cylindersInfo=[   50	  40	 30	     40	     30	      40	  40	  40;       %cylinder radius
                 100	 100	350     170     295	      20	  20	  20;       %cylinder axis end
                -121	-145	  0	    -20	      0	    -140	-140	-140;       %cylinder axis start
                   1	   2	  2	      3	      3	       4	   5	   6;       %cylinder subordinate to frame i
                   3	   3	  1	      3	      1	       3	   3	   3];      %cylinder axis orientation,1 means axis x,2 means axis y,3 means axis z

index = 4; % the target ball
[nodeList,isFind] = RRT(robot,initJointPos,ballsOrigin,ballsRadius,cylindersInfo,index);
filename = 'RRT.xlsx';
writematrix(nodeList,filename);


%% RRT
%%%% Function:RRT
function [nodeList,isFind]=RRT(robot,initJointPos,ballsOrigin,ballsRadius,cylindersInfo,index)

% Initialize
isFind = false;
targetPos=[0         0         1;
         0        -1         0;
         1         0         0];                                            % targetPos is the posture when manipulator touch balls
targetPos = [targetPos,[ballsOrigin(index,:)]';
                [0 0 0 1]];
targetJointPos = robot.ikine(targetPos);                                    % target joint pos                   

nodeList=[initJointPos,0,0];                                                % nodeList:col 1-6 is jointpos,col 7 is cost,col 8 is node parent 

angleStepSize= 2*pi/180;                                                    % joint space search step length is 2°
threhold=10;                                                                % distance between robot and  obstacle threhold is 10mm
maxLoopCount=15000;

% loop to grow RRTs
    for i=1:maxLoopCount
        newNode=SampleNearAndSteer(nodeList,angleStepSize,...
        initJointPos,targetJointPos);                                       % generate new node
        nearNode = nodeList(newNode(8),:);                                  % get nearest node
        noCollision=CheckPathCollision(newNode,nearNode,threhold,ballsOrigin,...
                                ballsRadius,cylindersInfo,index);           % check path collision             
        if noCollision
            nodeList=[nodeList;newNode];
        else
            continue;
        end
        
        isJointsPosNear = abs((targetJointPos-nearNode(1:6)))<...
        angleStepSize*5*[2 2 5 10 10 10];
        isNear = sum(isJointsPosNear);
        
        noCollision1=CheckPathCollision(newNode,targetJointPos,threhold,...
        ballsOrigin,ballsRadius,cylindersInfo,index);                       %check path collision
        if (isNear==6)&&noCollision1
            disp('find path successfully.')
            [n,~]=size(nodeList);
            nodeList=[nodeList;[targetJointPos,0,n]];
            isFind = true;
            return;
        end
        
    end
    isFind = false;
    disp('failed to find path.')
    
    [row1,~]=size(nodeList);
    len = zeros(row1,1);
    for i=1:row1
        len(i) = 100*(nodeList(i,1:3)-targetJointPos(1:3))*...
        (nodeList(i,1:3)-targetJointPos(1:3))'+...
        (nodeList(i,4:6)-targetJointPos(4:6))*(nodeList(i,4:6)-targetJointPos(4:6))';
    end
    [~,index]=min(len,[],1)
    nearestNode = nodeList(index,1:6)
    abs(targetJointPos - nearestNode)*180/pi
    disp(nodeList);
   
end

%%%% Get New Node
function newNode=SampleNearAndSteer(nodeList,angleStepSize,initJointPos,targetJointPos)
    Kp = 1.5;
    randNode = rand(1,6).*2*pi - pi;
    [row,~]=size(nodeList);
    len = zeros(row,1);
    for i=1:row
       len(i) = sqrt((nodeList(i,1:6)-randNode)*(nodeList(i,1:6)-randNode)');
    end
    [~,index]=min(len,[],1);
    nearNode = nodeList(index,1:6);
    
    randOrientation = (randNode-nearNode)/ sqrt((randNode-nearNode)*(randNode-nearNode)');
    searchOrientation=(targetJointPos-initJointPos)/ sqrt((targetJointPos-initJointPos)*(targetJointPos-initJointPos)');
   
    newNode = nearNode+angleStepSize*(randOrientation+Kp*searchOrientation);
    cost = sqrt((randOrientation+Kp*searchOrientation)*(randOrientation+Kp*searchOrientation)');
    newNode = [newNode,cost,index];
end


%% Check collision

%%%%%  Check path collison between newNode and nearNode

function noCollision=CheckPathCollision(newNode,nearNode,threhold,ballsOrigin,ballsRadius,cylindersInfo,index)
n=10;                                                                       %path interpolation count
initPos=newNode(1:6);
targetPos=nearNode(1:6);
for i=0:n
    pos = (targetPos-initPos)*i/n+initPos;
    noCollision=CheckLinksCollision(pos,threhold,ballsOrigin,ballsRadius,cylindersInfo,index);
end
end

%%%%%  Check manipulator links collision

function noCollision=CheckLinksCollision(q,threhold,ballsOrigin,ballsRadius,cylindersInfo,index)
% using SAT algorithm to check collision.
% The manipulator link is surrounded by cylinders
% obstacle is surrounded by balls
% links and balls project on the plane
    noCollision=true;
    [T01,T02,T03,T04,T05,T06]=RobotFkine(q);
    o=zeros(6,4);
    [row,~]=size(ballsOrigin);
    for i=1:row
        if(i~=index)
        o(1,:)=T01\[ballsOrigin(i,:),1]';                                                %ball origin in Coordinate 1
        o(2,:)=T02\[ballsOrigin(i,:),1]';                                                %ball origin in Coordinate 2
        o(3,:)=T03\[ballsOrigin(i,:),1]';                                                %ball origin in Coordinate 3
        o(4,:)=T04\[ballsOrigin(i,:),1]';                                                %ball origin in Coordinate 4
        o(5,:)=T05\[ballsOrigin(i,:),1]';                                                %ball origin in Coordinate 5
        o(6,:)=T06\[ballsOrigin(i,:),1]';                                                %ball origin in Coordinate 6
        o1=o(:,1:3);
            for j=1:8
                temp=CheckSingleLinkCollision(j,cylindersInfo,threhold,o1,ballsRadius(i));
                noCollision = noCollision&temp;
            end
        end
    end
end

%%%%%  Check single cylinder collision 

function noCollision=CheckSingleLinkCollision(idx,cylinderInfo,threhold,origin,ballRadius)

cylinderRadius = cylinderInfo(1,idx); 
cylinderAxisStart = cylinderInfo(3,idx); 
cylinderAxisEnd = cylinderInfo(2,idx); 
cylinderCoordinate = cylinderInfo(4,idx);
cylinderAxisOrientation = cylinderInfo(5,idx);
o=origin(cylinderCoordinate,:);

axiss=1:3;
axiss(cylinderAxisOrientation) =[];

if o(cylinderAxisOrientation)<cylinderAxisStart
    distance = sqrt((o(cylinderAxisOrientation)-cylinderAxisStart)^2+(sqrt(o(axiss(1))^2+o(axiss(2))^2)-cylinderRadius)^2);
elseif o(cylinderAxisOrientation)>cylinderAxisEnd
    distance = sqrt((o(cylinderAxisOrientation)-cylinderAxisEnd)^2+(sqrt(o(axiss(1))^2+o(axiss(2))^2)-cylinderRadius)^2);
else
    distance = sqrt(o(axiss(1))^2+o(axiss(2))^2)-cylinderRadius;
end

if distance>ballRadius+threhold
    noCollision = true;
else
    noCollision = false;
end
    
end


%% Other function： Forward kinemics of all joints
function [T01,T02,T03,T04,T05,T06] = RobotFkine(q)
d = [120.15 144.15 0 -142.64 113.5 113.5];   % d_i
a = [0 350 294.5 0 0 0];                     % a_i
alpha = [pi/2 0 0 -pi/2 pi/2 0];             % alpha_i
T01 = transl(0, 0, d(1)) * trotz(q(1));
T12 = trotx(alpha(1)) * transl(a(1), 0, 0) * transl(0, 0, d(2)) * trotz(q(2));
T23 = trotx(alpha(2)) * transl(a(2), 0, 0) * transl(0, 0, d(3)) * trotz(q(3));
T34 = trotx(alpha(3)) * transl(a(3), 0, 0) * transl(0, 0, d(4)) * trotz(q(4));
T45 = trotx(alpha(4)) * transl(a(4), 0, 0) * transl(0, 0, d(5)) * trotz(q(5));
T56 = trotx(alpha(5)) * transl(a(5), 0, 0) * transl(0, 0, d(6)) * trotz(q(6));
T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;
end