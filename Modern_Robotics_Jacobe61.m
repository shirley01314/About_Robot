function [J] = Modern_Robotics_Jacobe61(q)
%%现代机器人学例题6.1的雅克比矩阵的求解
zer = zeros(3,3);
%初始位形时，末端位姿M相对于基坐标系
M = [1 0 0 2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
w1 = [0;0;1];
w2 = [0;0;1];

q1 = [0;0;0];
q2 = [1;0;0];

v1 = -cross(w1,q1); %线速度
v2 = -cross(w2,q2);

R1 = [0 -w1(3) w1(2);w1(3) 0 -w1(1);-w1(2) w1(1) 0]; %转轴的反对称矩阵
R2 = [0 -w2(3) w2(2);w2(3) 0 -w2(1);-w2(2) w2(1) 0];

G1 = eye(3)*q(1) + (1-cos(q(1)))*R1 + (q(1)-sin(q(1)))*R1^2;  
G2 = eye(3)*q(2) + (1-cos(q(2)))*R2 + (q(2)-sin(q(2)))*R2^2;

Rot1 = eye(3) + sin(q(1))*R1 + (1-cos(q(1)))*R1^2;  %刚体转动的矩阵指数
Rot2 = eye(3) + sin(q(2))*R2 + (1-cos(q(2)))*R2^2; 

eS1 = [Rot1 G1*v1 ;0 0 0 1];%刚体运动的矩阵指数
eS2 = [Rot2 G2*v2 ;0 0 0 1];

Tsb = eS1*eS2*M;  %基于基坐标系的表示
%以上是正运动学的指数积公式

%以下是空间雅克比矩阵的求解
e_S1 = eS1;

e_S1_R = e_S1(1:3,1:3);

e_S1_P = e_S1(1:3,4);

e_S1_P_frame = [0 -e_S1_P(3) e_S1_P(2);e_S1_P(3) 0 -e_S1_P(1);-e_S1_P(2) e_S1_P(1) 0];

S1 = [w1;v1];
S2 = [w2;v2];

%伴随矩阵6*6
Ad_es1 = [e_S1_R  zer;e_S1_P_frame*e_S1_R e_S1_R];

Js1 = S1;
Js2 = Ad_es1*S2;

%空间雅克比的求解
Js = [Js1 Js2] ;%6*6 right
%物体雅克比的求解
Tbs = inv(Tsb);
R_Tbs = Tbs(1:3,1:3);
P_Tbs = Tbs(1:3,4);
P_Tbs_frame = [0 -P_Tbs(3) P_Tbs(2);P_Tbs(3) 0 -P_Tbs(1);-P_Tbs(2) P_Tbs(1) 0];
Ad_Tbs = [R_Tbs zer;P_Tbs_frame*R_Tbs R_Tbs];
Jb = Ad_Tbs * Js;

J = Jb;
end
