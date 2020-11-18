function [sys,x0,str,ts] = Plant_standardSfunction(t,x,u,flag)
switch flag
  case 0 %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1 %连续状态计算
    sys=mdlDerivatives(t,x,u);
  case {2,4,9} %离散状态计算，下一步仿真时刻，终止仿真设定
    sys=[];
  case 3 %输出信号计算
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 4;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 4;   %设置系统输入的变量
sizes.DirFeedthrough = 0;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [0.5  -0.5 0 0];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0


function sys=mdlDerivatives(t,x,u)  %该函数仅在连续系统中被调用，用于产生控制系统状态的导数
Uu = u(1);
Uv = u(2);
Uw = u(3);
TL = u(4);
iu=x(1);
iv=x(2);
iw=x(3);
dtheta=x(4);
%系统参数的定义
R = 5.6;
L = 11.57;
faif = 0.125;
Pn = 4;
J1 = 0.384*10e-4;
Tn = 1.47;
N = 4000;
Je = 1.25*10e-6;
J = J1+Je;
B1 = 0;
Be = 0.001;
B = B1+Be;
M = 1; %互感
xite = (L-M)*(L+2*M);
global theta;
theta = theta + dtheta * 0.0001;
a = faif * sin(theta);
b = faif * sin(theta-2*pi/3);
c = faif * sin(theta+2*pi/3);
T =[(-L*R-M*R)/xite  M*R/xite  M*R/xite (L*a+M*a-M*b-M*c)/xite;
    M*R/xite (-L*R-M*R)/xite M*R/xite (-M*a+L*b+M*b-M*c)/xite;
    M*R/xite M*R/xite (-L*R-M*R)/xite (-M*a-M*b+L*c+M*c)/xite;
    -a/J -b/J -c/J -B/J];  %4*4

amp = [(L*Uu+M*Uu-M*Uv-M*Uw)/xite; (-M*Uu+L*Uv+M*Uv-M*Uw)/xite; (-M*Uu-M*Uv+L*Uw+M*Uw)/xite; -TL/J];  %4*1
th = [iu; iv; iw; dtheta];  %4*1  iu iv iw dtheta

%运动方程
dth=T*th+amp;

sys(1)=dth(1);   
sys(2)=dth(2);   
sys(3)=dth(3);   
sys(4)=dth(4);  

function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
sys(1)=x(1);   %iu
sys(2)=x(2);   %iv
sys(3)=x(3);   %iw
sys(4)=x(4);   %dtheta



