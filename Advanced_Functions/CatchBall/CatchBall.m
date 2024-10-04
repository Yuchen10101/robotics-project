clear all;
clc;
%% 机器人建模
th(1) = 0; d(1) = 0.12015;   a(1) = 0;  alp(1) = pi/2;
th(2) = 0; d(2) = 0.14415;     a(2) = 0.360; alp(2) = 0;   
th(3) = 0; d(3) = 0;   a(3) = 0.3035;      alp(3) = 0;
th(4) = 0; d(4) = -0.14264;  a(4) = 0;      alp(4) = -pi/2;
th(5) = 0; d(5) = 0.1135;     a(5) = 0;      alp(5) = pi/2;
th(6) = 0; d(6) = 0.1135; a(6) = 0;      alp(6) = 0;
% DH parameters  th     d    a    alpha  sigma
L(1) = Link([th(1), d(1), a(1), alp(1), 0], 'revolute');
L(2) = Link([th(2), d(2), a(2), alp(2), 0], 'revolute');L(2).offset=pi/2;
L(3) = Link([th(3), d(3), a(3), alp(3), 0], 'revolute');
L(4) = Link([th(4), d(4), a(4), alp(4), 0], 'revolute');L(4).offset=-pi/2;
L(5) = Link([th(5), d(5), a(5), alp(5), 0], 'revolute');%L(5).offset=-pi;
L(6) = Link([th(6), d(6), a(6), alp(6), 0], 'revolute');
global robot
robot = SerialLink(L); %SerialLink 类函数
robot.name='robot';


%% 设定球的参数
g=0.5;
global time_step;
time_step=0.05;
%球的初速度、位置值
ball_initial_pos=[5,0,1];
velx=-1;
vely=0;
velz=1;

endtime=(velz+sqrt(velz*velz+2*g*(1+ball_initial_pos(3))))/g;%到地板（z=-1）停止
time_vec=0:time_step:endtime;
time_vec=transpose(time_vec);
ball_x=velx*time_vec+ball_initial_pos(1);
ball_y=vely*time_vec+ball_initial_pos(2);
ball_z=velz*time_vec-g/2*time_vec.^2+ball_initial_pos(3);


%% 求解机械臂接球轨迹
data=catchball(ball_x,ball_y,ball_z,time_vec);
if isempty(data)
   q_targ_t=zeros(length(time_vec),6,1);
   "Catch Ball Fail"
else
    "Catch Ball Success"
   q_targ_t=data(:,2:7);
   time_vec=data(:,1);
   
end


%% 画图
% 创建视频对象
video_filename = 'robot_catch_ball.mp4';
video = VideoWriter(video_filename, 'MPEG-4');
video.FrameRate = 15;
open(video);

fig=figure(1);
fig.Position=[100 100 1400 1200];
i=1;
radius=0.1;
[ball_x_mesh,ball_y_mesh,ball_z_mesh]=sphere;
ball_x_mesh=ball_x_mesh*radius;
ball_y_mesh=ball_y_mesh*radius;
ball_z_mesh=ball_z_mesh*radius;

v=[70 30];
w=[-3 7 -4 4 -1 4];
range_w=[w(2)-w(1),w(4)-w(3),w(6)-w(5)];
for i=1:1:length(time_vec)
 ball=surf(ball_x_mesh+ball_x(i),ball_y_mesh+ball_y(i),ball_z_mesh+ball_z(i),'EdgeColor','none'); 
 hold on;
 shooter=plot3(ball_initial_pos(1),ball_initial_pos(2),ball_initial_pos(3),'Marker','o','MarkerSize',15,'LineWidth',3);
 trace_ball=plot3(ball_x(1:i),ball_y(1:i),ball_z(1:i),'LineStyle','--','LineWidth',2);
 linez_ball=plot3([ball_x(i),ball_x(i)],[ball_y(i),ball_y(i)],[-1,ball_z(i)],'LineStyle','--');
 liney_ball=plot3([ball_initial_pos(1),ball_initial_pos(1)],[ball_initial_pos(2),ball_y(i)],[-1,-1],'LineStyle','--');
 linex_ball=plot3([ball_initial_pos(1),ball_x(i)],[ball_y(i),ball_y(i)],[-1,-1],'LineStyle','--');
 axis(w);
 pbaspect(range_w);
 robot.plot(q_targ_t(i,:),'tilesize',0.5,'workspace',w,'nowrist','view',v,'perspective');
 
 % Capture the plot as an image and add it to the video
 frame = getframe(gcf);
 writeVideo(video, frame);

 if i<length(time_vec)
 clf(fig)
 end
end
% 关闭视频对象
close(video);
%%
function data=catchball(ball_x,ball_y,ball_z,time_vec)
global robot;
global time_step;
N=length(time_vec);
q_init=[0 0 0 0 0 0];
iscatch=-1;
for i=1:1:N
    targ_p=[ball_x(i);ball_y(i);ball_z(i)];
    %首先根据工作空间排除达不到的点↓
    if not((targ_p(1)>-2) & (targ_p(1)<2) & (targ_p(2)>-2) & (targ_p(2)<2) & (targ_p(3)>-2) & (targ_p(3)<3))
        continue;
    end
    %排除达不到的点↓
    p=[0 0 -1 0;0 -1 0 0;-1 0 0 0;0 0 0 1];
    p(1:3,4)=targ_p;
    q_targ=ikine(robot,p);
    if(isempty(q_targ))
        continue;
    end
    %排除时间层面上来不及达到的点↓
    time=0.8;
    qd_limit=2.2;
    robot_time_vec=0:time_step:time;
    [q_targ_t ,qd_t, qdd_t]=jtraj(q_init,q_targ,robot_time_vec);

    while(max(abs(qd_t),[],'all')>qd_limit)
        time=time*max(abs(qd_t),[],'all')/qd_limit+time_step;
        robot_time_vec=0:time_step:time;
        [q_targ_t ,qd_t, qdd_t]=jtraj(q_init,q_targ,robot_time_vec);
    end
    %可以达到的点↓
    if time<=time_vec(i)
        iscatch=1;
        break;
    end
end
%输出结果↓
if iscatch==-1
    data=[];
else
    n=length(robot_time_vec);
    data=zeros(i,7,1);
    data(:,1)=time_vec(1:i);
    data(i-n+1:i,2:7)=q_targ_t;
    
end
end

