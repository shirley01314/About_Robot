function [J] = Modern_Robotics_Jacobe52(q)
%现代机器人学例题5.2的雅克比矩阵的验证
zer = zeros(3,3);
%初始位形时，末端位姿M相对于基坐标系
syms th1 th2 th3 L0 L1 L2
M = [1 0 0 L1+L2;
     0 1 0 0;
     0 0 1 L0;
     0 0 0 1];
w1 = [0;0;1];
w2 = [0;0;1];
w3 = [0;0;1];

q1 = [0;0;0];
q2 = [L1;0;0];
q3 = [L1+L2;0;0];

v1 = -cross(w1,q1); %线速度
v2 = -cross(w2,q2);
v3 = -cross(w3,q3);

R1 = [0 -w1(3) w1(2);w1(3) 0 -w1(1);-w1(2) w1(1) 0]; %转轴的反对称矩阵
R2 = [0 -w2(3) w2(2);w2(3) 0 -w2(1);-w2(2) w2(1) 0];
R3 = [0 -w3(3) w3(2);w3(3) 0 -w3(1);-w3(2) w3(1) 0];

G1 = eye(3)*th1 + (1-cos(th1))*R1 + (th1-sin(th1))*R1^2;  
G2 = eye(3)*th2 + (1-cos(th2))*R2 + (th2-sin(th2))*R2^2;
G3 = eye(3)*th3 + (1-cos(th3))*R3 + (th3-sin(th3))*R3^2;

Rot1 = eye(3) + sin(th1)*R1 + (1-cos(th1))*R1^2;  %刚体转动的矩阵指数
Rot2 = eye(3) + sin(th2)*R2 + (1-cos(th2))*R2^2; 
Rot3 = eye(3) + sin(th3)*R3 + (1-cos(th3))*R3^2;

eS1 = [Rot1 G1*v1 ;0 0 0 1];%刚体运动的矩阵指数
eS2 = [Rot2 G2*v2 ;0 0 0 1];
eS3 = [Rot3 G3*v3 ;0 0 0 1];

Tsb = eS1*eS2*eS3*M;  %基于基坐标系的表示

%以上是正运动学的指数积公式
%以下是空间雅克比矩阵的求解
e_S1 = eS1;
e_S2 = eS1*eS2;

e_S1_R = e_S1(1:3,1:3);
e_S2_R = e_S2(1:3,1:3);

e_S1_P = e_S1(1:3,4);
e_S2_P = e_S2(1:3,4);

e_S1_P_frame = [0 -e_S1_P(3) e_S1_P(2);e_S1_P(3) 0 -e_S1_P(1);-e_S1_P(2) e_S1_P(1) 0];
e_S2_P_frame = [0 -e_S2_P(3) e_S2_P(2);e_S2_P(3) 0 -e_S2_P(1);-e_S2_P(2) e_S2_P(1) 0];

S1 = [w1;v1];
S2 = [w2;v2];
S3 = [w3;v3];

%伴随矩阵6*6
Ad_es1 = [e_S1_R  zer;e_S1_P_frame*e_S1_R e_S1_R];
Ad_es1es2 = [e_S2_R  zer;e_S2_P_frame*e_S2_R e_S2_R];

Js1 = S1;
Js2 = Ad_es1*S2;
Js3 = Ad_es1es2*S3;

%空间雅克比的求解
Js = [Js1 Js2 Js3]  %6*6
%物体雅克比的求解
Tbs = inv(Tsb);
R_Tbs = Tbs(1:3,1:3);
P_Tbs = Tbs(1:3,4);
P_Tbs_frame = [0 -P_Tbs(3) P_Tbs(2);P_Tbs(3) 0 -P_Tbs(1);-P_Tbs(2) P_Tbs(1) 0];
Ad_Tbs = [R_Tbs zer;P_Tbs_frame*R_Tbs R_Tbs];
Jb = Ad_Tbs * Js;


J = subs(Jb,{th1,th2,th3},{q(1),q(2),q(3)});
J = eval(J);
end
