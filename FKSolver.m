function [ Tout ] = FKSolver( q )
%正运动学
%输入q为六个关节角(单位：弧度)，计算末端位姿变换矩阵  
Tx = [0, -0.1573, -0.0165, -0.2975, -0.3545, 0.0820];%Tx = [0, 0.1949, 1.0938e-04, 0.2000, 0, -4.7162e-04];
Ty = [0, 0.0653, 0.0464, -0.0519, 0.0440, -0.0145];%Ty = [0, -0.0951, -0.6137, 0.2750, 0.0320, 0.0981];
Tz = [0, 0, 0.6136, 0.1671, -0.0437, -0.5149];%Tz = [0, 0, 0.0030, 0.1105, 0.3650, 0.0540];
alpha = [0, -pi/2, 0, -pi/2, pi/2, -pi/2];
syms th1 th2 th3 th4 th5 th6;
%每个关节相对于前一关节坐标系的变换矩阵
T{1} = transl(Tx(1), Ty(1), Tz(1))*trotx(alpha(1))*trotz(th1);
T{2} = transl(Tx(2), Ty(2), Tz(2))*trotx(alpha(2))*trotz(th2);
T{3} = transl(Tx(3), Ty(3), Tz(3))*trotx(alpha(3))*trotz(th3);
T{4} = transl(Tx(4), Ty(4), Tz(4))*trotx(alpha(4))*trotz(th4);
T{5} = transl(Tx(5), Ty(5), Tz(5))*trotx(alpha(5))*trotz(th5);
T{6} = transl(Tx(6), Ty(6), Tz(6))*trotx(alpha(6))*trotz(th6);
%末端执行器相对于惯性系0的变换矩阵
T06 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};

Tout= subs(T06, {th1, th2, th3, th4, th5, th6},{q(1)-pi/2, q(2), q(3), q(4)-pi/2, q(5), q(6)});
Tout = eval(Tout);


end