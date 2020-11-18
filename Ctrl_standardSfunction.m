function [sys,x0,str,ts] = Ctrl_standardSfunction(t,x,u,flag)
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case {2,4,9}
    sys=[];
  case 3
    sys=mdlOutputs(t,x,u);
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
function [sys,x0,str,ts]=mdlInitializeSizes   %系统的初始化
sizes = simsizes;
sizes.NumContStates  = 0;   %设置系统连续状态的变量
sizes.NumDiscStates  = 0;   %设置系统离散状态的变量
sizes.NumOutputs     = 4;   %设置系统输出的变量
sizes.NumInputs      = 4;   %设置系统输入的变量
sizes.DirFeedthrough = 1;   %如果在输出方程中显含输入变量u，则应该将本参数设置为1
sizes.NumSampleTimes = 0;   % 模块采样周期的个数
                            % 需要的样本时间，一般为1.
                            % 猜测为如果为n，则下一时刻的状态需要知道前n个状态的系统状态
sys = simsizes(sizes);
x0  = [];                   % 系统初始状态变量
str = [];                   % 保留变量，保持为空
ts  = [];                   % 采样时间[t1 t2] t1为采样周期，如果取t1=-1则将继承输入信号的采样周期；参数t2为偏移量，一般取为0
 
    
function sys=mdlOutputs(t,x,u)   %产生（传递）系统输出
T1=-f/g;
T2=(a1*E5*th^(E5-1)*dth^2*k^2-E2*k^(1-q1/p1)*dth^(2-q1/p1))/(g*k);
 if dth^(q1/p1-1)<=tao;
    miu=sin(pi/2*dth^(q1/p1-1)/tao);
 else 
     miu=1;
 end
T3=-1/g*E2*k^(-q1/p1)*(a2*s^E3+belta2*s^E4)*dth^(1-q1/p1)*miu;
ut=T1+T2+T3;

sys(1)=ut;

