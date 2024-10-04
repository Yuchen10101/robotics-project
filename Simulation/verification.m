%g0
g0=-9.80665*[0;0;1]; %m/s^2
%damp_coeff
damp_coeff=[4 4 4 4 4 4]; %N*m/(rad/s)

Para=[15.135 0 0 0 [0.044302, 0.043091, 0.030523] [1.4326e-05, -6.1966e-07, 1.5349e-07];
    5.459 0 0 0 [0.012901, 0.14154, 0.13805] [1.8902e-08, -9.548800000000001e-06, -1.8956e-07]
    2.1077 0 0 0 [0.0026619, 0.036078, 0.035426] [-1.3969e-08, -0.0023256, -4.7377e-07]
    5.5436 0 0 0 [0.0092224, 0.0052149, 0.0089344] [-8.4855e-05, -1.2691e-08, 1.1405e-07]
    6.3339 0 0 0 [0.012993, 0.0058454, 0.0127] [4.5426e-05, 5.8559e-09, -1.0764e-07]
    1.4169 0 0 0 [0.00071853, 0.00072375, 0.0011337] [8.3072e-07, -6.378e-07, -6.9778e-07]];

%% 1.仿真结束后，导入传感器数据
rawdata=readtable("D:\J\Downloads\Undergraduate\Robotics\robot\result1.xlsx");
% 仿真10s
t=rawdata(:,"time"); %s
time=linspace(0,30,size(rawdata,1));
tau_sm=rawdata(:,["t1" "t2" "t3" "t4" "t5" "t6"]); %N*m
th=rawdata(:,["q1" "q2" "q3" "q4" "q5" "q6"]); %rad
dth=rawdata(:,["dq1" "dq2" "dq3" "dq4" "dq5" "dq6"]); %rad/s
ddth=rawdata(:,["ddq1" "ddq2" "ddq3" "ddq4" "ddq5" "ddq6"]); %rad/s^2
% 格式转换
t=table2array(t);
tau_sm=table2array(tau_sm);
th=table2array(th);
dth=table2array(dth);
ddth=table2array(ddth);

% 去掉阻尼项
for i=1:6
    tau_sm(:,i) = tau_sm(:,i) - damp_coeff(i)*dth(:,i);
end

% 用于初始位置的MDH设置
    nu=6;
    R=zeros(3,3,nu);P=zeros(3,nu);
    %MDH为[a_(i-1),alpha_(i-1),d_i,theta_i]
    MDH=[0 0 0.12 0;
        0 pi/2 0.144 0;
        0.35 0 0 0;
        0.294 0 -0.029 0;
        0 -pi/2 0.113 0;
        0 pi/2 0.107 0];

    %根据MDH得到各个齐次转换矩阵
    %R(:,:,i)=R^(i-1)_i, eg: R(:,:,1)=R^0_1 , R(:,:,6)=R^5_6
    %P(:,i)=r^(i-1)_(i-1,i), eg: P(:,1)=r^0_(0,1), P(:,6)=r^5_(5,6)
    for i=1:6
        [R(:,:,i),P(:,i)]=cal_R(MDH(i,:));
    end

%% 2.惯性参数处理
PARAM=inertial_para_process(Para);

%% 3.计算预测力矩
tau_es=zeros(size(time,2),6);
for i=1:size(time,2)
    ii=i;
    tau_es(ii,:)=estimation(th(ii,:),dth(ii,:),ddth(ii,:),PARAM,g0);
end

%% 4.输出6个关节力矩曲线
for j = 1:6 
    subplot(3,2,j);
    hold on
    plot(time,tau_es(:,j),'LineWidth',1);
    plot(time,tau_sm(:,j),'--','LineWidth',2);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    hold off
end
legend('ES','SM');
%% 5.计算RMS
RMS=zeros(6,1);
for i=1:6
    RMS(i)=rms(tau_es(:,i)-tau_sm(:,i)); 
end

%% 6.预测函数的实现
function tau = estimation(th,dth,ddth,PARAM,g0)
%th,dth,ddth: 6x1 double
    nu=6;
    ro=Robotfunction;
    R=zeros(3,3,nu); P=zeros(3,nu);   IR=R;
    %MDH为[a_(i-1),alpha_(i-1),d_i,theta_i]
    MDH=[0 0 0.12 th(1);  0 pi/2 0.144 th(2);  0.35 0 0 th(3); 
        0.294 0 -0.029 th(4);0 -pi/2 0.113 th(5); 0 pi/2 0.107 th(6)];
    %齐次变换矩阵计算
    for i=1:6
        [R(:,:,i),P(:,i)]=cal_R(MDH(i,:));
        IR(:,:,i)=R(:,:,i).';%IR即R的逆
    end

    % 运动学正推,计算omega^i_i, v^i_i, dv^i_i, domega^i_i, i=1:6
    w0=[0;0;0];dw0=[0;0;0];dv0=[0;0;0]; 
    Z=[0;0;1];
    w(:,1) = IR(:,:,1)*w0+dth(1)*Z;
    dw(:,1) = IR(:,:,1)*dw0+cross(IR(:,:,1)*w0,(dth(1)*Z))+ddth(1)*Z;
    dv(:,1) = IR(:,:,1)*(cross(dw0,P(:,1))+cross(w0,cross(w0,P(:,1)))+dv0) ;
    for i=2:nu
        w(:,i) = IR(:,:,i)*w(:,i-1)+dth(i)*Z;
        dw(:,i) = IR(:,:,i)*dw(:,i-1)+cross(IR(:,:,i)*w(:,i-1),(dth(i)*Z))+ddth(i)*Z;
        dv(:,i) = IR(:,:,i)*(cross(dw(:,i-1),P(:,i))+cross(w(:,i-1),cross(w(:,i-1),P(:,i)))+dv(:,i-1));  
    end

    % 计算力矩
    tau = [0 0 0 0 0 0];
    for i = 1 : nu
        for k = i : nu
            tau(i)=tau(i) + Z'* ro.R(R,i,k) * ro.S( ro.R(R,0,k).'*g0 - dv(:,k) ) * PARAM(k,2:4).';
            tau(i)=tau(i) + Z'* ro.R(R,i,k) * ro.M(w(:,k),dw(:,k)) * PARAM(k,5:10).';
        end
        for k = i : nu-1
            for j = k+1 : nu
                tau(i)=tau(i) + Z'* ro.S(ro.R(R,i,k)*P(:,k+1)) * ro.R(R,i,j) * (- ro.R(R,0,j).'*g0 + dv(:,j)) *PARAM(j,1);
                tau(i)=tau(i) + Z'* ro.S(ro.R(R,i,k)*P(:,k+1)) * ro.R(R,i,j) * ro.N(w(:,j),dw(:,j)) * PARAM(j,2:4).';
            end
        end
    end
end

%% 7.惯性参数处理函数的实现
function PARAM = inertial_para_process(Para)
%输入排列顺序：1m:kg 2x:m 3y:m 4z:m 5xx:kg*m^2 6yy 7zz 8yz 9xz 10xy
%给模型的二阶张量参考点是质心，但质心坐标、二阶张量参考坐标系都是{i}系，所以必须要用平行轴定理
%输入m x y z I^i_(C_i)
%输出m mx my mz I^i_i
%输出m mx my mz xx XY XZ YY YZ ZZ
% 平行轴定理 + 重新排序+质心位置变成一阶惯性张量
    PARAM=Para;
    I=zeros(6,3,3);
    for i=1:6
        I(i,1,1)=PARAM(i,5);
        I(i,2,2)=PARAM(i,6);
        I(i,3,3)=PARAM(i,7);
        I(i,1,2)=PARAM(i,10); %xy
        I(i,2,1)=PARAM(i,10);
        I(i,1,3)=PARAM(i,9); %xz
        I(i,3,1)=PARAM(i,9);
        I(i,2,3)=PARAM(i,8); %yz
        I(i,3,2)=PARAM(i,8);
    end
    
    %平行轴定理
    for i=1:6
        r=PARAM(i,2:4).';
        I_temp=squeeze(I(i,:,:));
        I_temp=I_temp+PARAM(i,1)*(r'*r*eye(3) - r*r');
        I(i,1:3,1:3)=I_temp(1:3,1:3);
    end
    
    %现有顺序5xx 6yy 7zz 8yz 9xz 10xy
    %目标需要5xx 10xy 9xz 6yy 8yz 7zz,
    % PARAM(:,[5 6 7 8 9 10])=PARAM(:,[5 10 9 6 8 7]);
    for i=1:6
        PARAM(i,5)=I(i,1,1);
        PARAM(i,6)=I(i,1,2);
        PARAM(i,7)=I(i,1,3);
        PARAM(i,8)=I(i,2,2);
        PARAM(i,9)=I(i,2,3);
        PARAM(i,10)=I(i,3,3);
    end
    %质心坐标变为一阶惯性张量
    PARAM(:,2:4)=PARAM(:,2:4).*PARAM(:,1);
end

%% 各个矩阵的函数实现
function Robot =Robotfunction
    Robot.L=@L_trans;
    Robot.S=@S_trans;
    Robot.M=@M_trans;
    Robot.N=@N_trans;
    Robot.R=@R_trans;
    Robot.P=@P_trans;
end

function [Lx]=L_trans(x)
    Lx=[x(1),x(2),x(3),0,0,0
        0,x(1),0,x(2),x(3),0
        0,0,x(1),0,x(2),x(3)];
end

function [Sx]=S_trans(x)
    Sx=[0,-x(3),x(2);x(3),0,-x(1);-x(2),x(1),0];
end

function [M]=M_trans(w,dw) 
%return beta,M与惯性张量结合
    M=L_trans(dw)+S_trans(w)*L_trans(w);
end

function [N]=N_trans(w,dw) 
%return gamma, N与一阶惯性张量结合
    N=S_trans(dw)+S_trans(w)*S_trans(w);
end

function [MR]=R_trans(R,i,j) %j相对i的坐标变换
%return R^i_j
MR=eye(3);

flag = 0;
if(i>j)
    temp=i;
    i=j;
    j=temp;
    flag = 1;
end

while i<j
    MR=MR*R(:,:,i+1);
    i=i+1;
end

if flag==0
    return;
else
    MR=MR';
end
end

function [MP]=P_trans(P,R,i,j) %实际得到r^i_(i,j+1),但希望得到r^i_(j,j+1)
%return r^i_(i,j+1)
    MP=P(:,i+1);
    while i<j
        MP=MP+R_trans(R,i,j)*P(:,j+1);
        j=j-1;
    end
end

%% 齐次变换矩阵函数实现
function [R,p] = cal_R(MDH)
%计算齐次变换矩阵T^(i-1)_i, 传入的MDH为[a_(i-1),alpha_(i-1),d_i,theta_i]
%根据MDH得到各个齐次转换矩阵
%R(:,:,i)=R^(i-1)_i, eg: R(:,:,1)=R^0_1 , R(:,:,6)=R^5_6
%P(:,i)=r^(i-1)_(i-1,i), eg: P(:,1)=r^0_(0,1), P(:,6)=r^5_(5,6)
    R=[cos(MDH(4)),            -sin(MDH(4)),           0;
       sin(MDH(4))*cos(MDH(2)), cos(MDH(4))*cos(MDH(2)), -sin(MDH(2));
       sin(MDH(4))*sin(MDH(2)), cos(MDH(4))*sin(MDH(2)), cos(MDH(2))];
    p=[MDH(1);-sin(MDH(2))*MDH(3);cos(MDH(2))*MDH(3)];
end