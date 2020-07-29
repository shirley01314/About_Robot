function [ J ] = Jacob(th, T_d)
%计算末端相对于0坐标系的雅可比矩阵
%   此处显示详细说明
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
%各个关节相对于惯性系0的变换矩阵
T01 = T{1};
T02 = T{1}*T{2};
T03 = T{1}*T{2}*T{3};
T04 = T{1}*T{2}*T{3}*T{4};
T05 = T{1}*T{2}*T{3}*T{4}*T{5};
T06 = T{1}*T{2}*T{3}*T{4}*T{5}*T{6};

f1 = T06(1, 4) - T_d(1, 4);
f2 = T06(2, 4) - T_d(2, 4);
f3 = T06(3, 4) - T_d(3, 4);
f4 = T06(2, 3) - T_d(2, 3);
f5 = T06(3, 3) - T_d(3, 3);
f6 = T06(3, 2) - T_d(3, 2);
f7 = T06(1, 1) - T_d(1, 1);
f8 = T06(1, 2) - T_d(1, 2);
f9 = T06(1, 3) - T_d(1, 3);
f10 = T06(2, 1) - T_d(2, 1);
f11 = T06(2, 2) - T_d(2, 2);
f12 = T06(3, 1) - T_d(3, 1);

J = [diff(f1, th1), diff(f1, th2), diff(f1, th3), diff(f1, th4), diff(f1, th5), diff(f1, th6);
    diff(f2, th1), diff(f2, th2), diff(f2, th3), diff(f2, th4), diff(f2, th5), diff(f2, th6);
    diff(f3, th1), diff(f3, th2), diff(f3, th3), diff(f3, th4), diff(f3, th5), diff(f3, th6);
    diff(f4, th1), diff(f4, th2), diff(f4, th3), diff(f4, th4), diff(f4, th5), diff(f4, th6);
    diff(f5, th1), diff(f5, th2), diff(f5, th3), diff(f5, th4), diff(f5, th5), diff(f5, th6);
    diff(f6, th1), diff(f6, th2), diff(f6, th3), diff(f6, th4), diff(f6, th5), diff(f6, th6);
    diff(f7, th1), diff(f7, th2), diff(f7, th3), diff(f7, th4), diff(f7, th5), diff(f7, th6);
    diff(f8, th1), diff(f8, th2), diff(f8, th3), diff(f8, th4), diff(f8, th5), diff(f8, th6);
    diff(f9, th1), diff(f9, th2), diff(f9, th3), diff(f9, th4), diff(f9, th5), diff(f9, th6);
    diff(f10, th1), diff(f10, th2), diff(f10, th3), diff(f10, th4), diff(f10, th5), diff(f10, th6);
    diff(f11, th1), diff(f11, th2), diff(f11, th3), diff(f11, th4), diff(f11, th5), diff(f11, th6);
    diff(f12, th1), diff(f12, th2), diff(f12, th3), diff(f12, th4), diff(f12, th5), diff(f12, th6)];

J = subs(J, {th1, th2, th3, th4, th5, th6},{th(1)-pi/2, th(2), th(3), th(4)-pi/2, th(5), th(6)});
J = eval(J);



end

