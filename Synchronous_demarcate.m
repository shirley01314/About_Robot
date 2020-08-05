%此程序为辨识电机参数的定子电阻Rs, 永磁体磁通faif, dq轴电感L
function [sys,x0,str,ts] = Synchronous_demarcate(t,x,u,flag)
switch flag,
  case 0, %初始化
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 2 %离散状态计算，下一步仿真时刻，终止仿真设定
    sys=[];%mdlUpdates(t,x,u);
  case 3, %输出信号计算
    sys=mdlOutputs(t,x,u);
  case {1,4,9}, %输出信号计算
    sys=[];
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 0;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 3;   %设置系统输出的变量
sizes.NumInputs      = 5;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1,输入不直接传到输出口
sizes.NumSampleTimes = 1;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];            % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [0 0];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0

global  P_past theta_past
P_past = 1e4 * eye(3,3);
theta_past = [0;0;10000];

function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
%初值的确定
lambda = 1;  %遗忘因子， 选择不遗忘
global   P_past theta_past
id = u(1);
iq = u(2);
w = u(3);
diq = u(4);
uq = u(5);
xt = [-iq -w uq];
y = diq+w*id;
P_new = 1/lambda*(P_past-(P_past*(xt'*xt)*P_past)/(lambda+xt*P_past*xt'));
L = P_past*xt'/(lambda+xt*P_past*xt');
theta_new = theta_past + L*(y-xt*theta_past);
a = theta_new(1);
b = theta_new(2);
c = theta_new(3);
R = a/c;
Ke = b/c;
Ld = 1/c;
P_past = P_new ;
theta_past = theta_new;
sys(1) = R;
sys(2) = Ke;
sys(3) = Ld;






