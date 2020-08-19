function [sys,x0,str,ts] = Newton_EulerDynamics(t,x,u,flag)
switch flag,
  case 0, %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1, %连续状态计算
    sys=mdlDerivatives(t,x,u);
  case {2,4,9}, %离散状态计算，下一步仿真时刻，终止仿真设定
    sys=[];
  case 3, %输出信号计算
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 0;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 1;   %设置系统输出的变量
sizes.NumInputs      = 18;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
global torque6 torque
torque6 = 0;
torque = 0;

function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
syms q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6
global torque6 torque
if t==0
    th1 = u(1);
    th2 = u(2);
    th3 = u(3);
    th4 = u(4);
    th5 = u(5);
    th6 = u(6);

    dth1 = u(7);
    dth2 = u(8);
    dth3 = u(9);
    dth4 = u(10);
    dth5 = u(11);
    dth6 = u(12);

    ddth1 = u(13);
    ddth2 = u(14);
    ddth3 = u(15);
    ddth4 = u(16);
    ddth5 = u(17);
    ddth6 = u(18);
    dh_parms = [0,0,0,q1;
                -pi/2,0.3581,0,q2;
                0,0.6137,0,q3-pi/2;
                -pi/2,0.3575,0,q4;
                pi/2,0.3664,0,q5;
                -pi/2,0.12,0,q6;
                0,0,0,0];
    mass_center = [0.0467, -0.0104, 0.1100;
                   -0.0155, -0.2895, -0.0268;
                   0.1366, 0.1311, 0.0806;
                   0.0350, 0.0044, 0.1474;
                   3.9663e-06, -0.0019, 0.0540;
                   0, 0, 0.05];
    mass  = [17.9513, 3.5484, 7.3201, 3.8682, 0.7287, 1]; %六个连杆的质量]
    inertia_1 = [ 0.5354,0.0131,-0.2059;
                0.0131,0.7118,0.0404;
                -0.2059,0.0404,0.5010];
    inertia_2 = [0.5044, -0.0164, -0.0021;
        -0.0164, 0.0144, -0.0304;
        -0.0021, -0.0304, 0.5091];
    inertia_3 = [0.2601, -0.1844, -0.0883;
        -0.1844, 0.2780, -0.0850;
        -0.0883, -0.0850, 0.3979];
    inertia_4 = [0.1544, -0.0001, -0.0143;
        -0.0001, 0.1527, -0.0051;
        -0.0143, -0.0051, 0.0224];    
    inertia_5 = [0.0055, 0, 0;
        0, 0.0040, -0.0015;
        0, -0.0015, 0.0028]; 
    inertia_6 = [0.0042, 0, 0;
            0, 0.0042, 0;
            0, 0, 0.02];      

    inertia_tensor(:,:,1) = inertia_1;
    inertia_tensor(:,:,2) = inertia_2;
    inertia_tensor(:,:,3) = inertia_3;
    inertia_tensor(:,:,4) = inertia_4;
    inertia_tensor(:,:,5) = inertia_5;
    inertia_tensor(:,:,6) = inertia_6;
    f_ext = [0,0,0;
             0,0,0;
             0,0,0;
             0,0,0;
             0,0,0;
             0,0,0];
    torque = NewtonEulerDynamics(dh_parms, mass, mass_center, inertia_tensor, f_ext); 
    torque6 = subs(torque,{q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6},{th1,th2,th3,th4,th5,th6,dth1,dth2,dth3,dth4,dth5,dth6,ddth1,ddth2,ddth3,ddth4,ddth5,ddth6});
    sys(1) = torque6;
end
th1 = u(1);
th2 = u(2);
th3 = u(3);
th4 = u(4);
th5 = u(5);
th6 = u(6);

dth1 = u(7);
dth2 = u(8);
dth3 = u(9);
dth4 = u(10);
dth5 = u(11);
dth6 = u(12);

ddth1 = u(13);
ddth2 = u(14);
ddth3 = u(15);
ddth4 = u(16);
ddth5 = u(17);
ddth6 = u(18);
% torque1 = torque(1);
% torque2 = torque(2);
% torque3 = torque(3);
% torque4 = torque(4);
% torque5 = torque(5);
% torque6 = ddq6/200 + (ddq4*cos(q5))/200 %- (dq4*dq5*sin(q5))/200 + (ddq2*sin(q4)*sin(q5))/200 + (ddq3*sin(q4)*sin(q5))/200 - (ddq1*cos(q2)*cos(q3)*cos(q5))/200 + (ddq1*cos(q5)*sin(q2)*sin(q3))/200 + (dq2*dq4*cos(q4)*sin(q5))/200 + (dq2*dq5*cos(q5)*sin(q4))/200 + (dq3*dq4*cos(q4)*sin(q5))/200 + (dq3*dq5*cos(q5)*sin(q4))/200 + (ddq1*cos(q2)*cos(q4)*sin(q3)*sin(q5))/200 + (ddq1*cos(q3)*cos(q4)*sin(q2)*sin(q5))/200 + (dq1*dq2*cos(q2)*cos(q5)*sin(q3))/200 + (dq1*dq2*cos(q3)*cos(q5)*sin(q2))/200 + (dq1*dq3*cos(q2)*cos(q5)*sin(q3))/200 + (dq1*dq3*cos(q3)*cos(q5)*sin(q2))/200 + (dq1*dq5*cos(q2)*cos(q3)*sin(q5))/200 - (dq1*dq5*sin(q2)*sin(q3)*sin(q5))/200 + (dq1*dq2*cos(q2)*cos(q3)*cos(q4)*sin(q5))/200 + (dq1*dq3*cos(q2)*cos(q3)*cos(q4)*sin(q5))/200 + (dq1*dq5*cos(q2)*cos(q4)*cos(q5)*sin(q3))/200 + (dq1*dq5*cos(q3)*cos(q4)*cos(q5)*sin(q2))/200 - (dq1*dq2*cos(q4)*sin(q2)*sin(q3)*sin(q5))/200 - (dq1*dq3*cos(q4)*sin(q2)*sin(q3)*sin(q5))/200 - (dq1*dq4*cos(q2)*sin(q3)*sin(q4)*sin(q5))/200 - (dq1*dq4*cos(q3)*sin(q2)*sin(q4)*sin(q5))/200;
torque6 = subs(torque,{q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6},{th1,th2,th3,th4,th5,th6,dth1,dth2,dth3,dth4,dth5,dth6,ddth1,ddth2,ddth3,ddth4,ddth5,ddth6});
sys(1) = torque6;






