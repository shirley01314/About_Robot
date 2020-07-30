% 此程序为DH参数的标定
% 注：该DH参数的标定由于使用的是toolbox里的SerialLink，它里面对于DH参数的使用本人没有去深究是怎样建立连杆的，通过程序实验，该DH参数能否标定正确与连杆转角alpha有很大的关系
syms a1 a2 a3 a4 a5 a6; %连杆长度
syms d1 d2 d3 d4 d5 d6; %连杆偏距
syms q1 q2 q3 q4 q5 q6; %关节角度
alpha = [pi/2;pi/2;-pi/2;pi/2;-pi/2;0];%连杆转角
links(1) = Link ([0,d1,a1,alpha(1),0]);
links(2) = Link ([0,d2,a2,alpha(2),0]);
links(3) = Link ([0,d3,a3,alpha(3),0]);
links(4) = Link ([0,d4,a4,alpha(4),0]);
links(5) = Link ([0,d5,a5,alpha(5),0]);
links(6) = Link ([0,d6,a6,alpha(6),0]);
KR5_i= SerialLink(links, 'name', 'Kuka KR5') ;
T=KR5_i.fkine([q1;q2;q3;q4;q5;q6]);  %4*4矩阵
P = T * [0;0;0;1]; %在基座坐标系下的坐标P
Tx=P(1,:);  
Ty=P(2,:);
Tz=P(3,:);
Lx = [diff(Tx,a1), diff(Tx,a2), diff(Tx,a3), diff(Tx,a4), diff(Tx,a5), diff(Tx, a6), diff(Tx,d1), diff(Tx,d2), diff(Tx, d3), diff(Tx, d4), diff(Tx, d5), diff(Tx, d6)]; 
Ly = [diff(Ty,a1), diff(Ty,a2), diff(Ty,a3), diff(Ty,a4), diff(Ty,a5), diff(Ty, a6), diff(Ty,d1), diff(Ty,d2), diff(Ty, d3), diff(Ty, d4), diff(Ty, d5), diff(Ty, d6)]; 
Lz = [diff(Tz,a1), diff(Tz,a2), diff(Tz,a3), diff(Tz,a4), diff(Tz,a5), diff(Tz, a6), diff(Tz,d1), diff(Tz,d2), diff(Tz, d3), diff(Tz, d4), diff(Tz, d5), diff(Tz, d6)]; 
L  = [Lx;Ly;Lz];   %L为偏导系数矩阵;  3*12
delta = zeros(12,6);

%DH参数的理论值
a1_N = 0.18;
a2_N = 0.6;
a3_N = 0.12;
a4_N = 0.05;
a5_N = 0.2;
a6_N = 0.01;
d1_N = 0.4;
d2_N = 0;
d3_N = 1;
d4_N = 0.62;
d5_N = 0;
d6_N = 0.1;

%设为实际DH参数 与理想DH参数有小的偏差
%在选择DH参数时，也应该尽量不要选择相同的a或相同的d，否则很可能迭代不出来正确的a和d
a1_true = 0.3;
a2_true = 0.7;
a3_true = 0.15;
a4_true = 0.1;
a5_true = 0.3;
a6_true = 0.05;
d1_true = 0.5;
d2_true = 1;
d3_true = 0.5;
d4_true = 0.7;
d5_true = 0.1;
d6_true = 0.2;

% 注：更改角度值，尽量不要取相等的关节角， 不同的组的关节角应该尽量相差大一点，一组的关节角角度也相差大一点
q1_real=[-0.1,-0.2,-0.3,-0.4,-0.5,-0.6];  %定义m = 9 九组测得的关节角，算出理论上的坐标;
q2_real=[1.8,2.8,3.8,-0.5,-1,-1.7];
q3_real=[1.5,0.5,2.0,0.5,1.0,1.5];
q4_real=[-1.5,0.5,-1,2.1,-0.3,1];
q5_real=[-1;-2;-3;-4;-5;-6];
q6_real=[0.6,0.7,0.8,0.9,1.0,1.1];
q7_real=[0.8,0.6,1.0,-1.0,-0.5,-0.7];
q8_real=[0.8,0.9,1.0,1.1,1.2,1.3];
q9_real=[-0.9,-1.0,1.1,-1.2,1.3,-1.4];

%产生实际坐标
disp('实际坐标开始产生')
TT1 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT1 = (subs(TT1,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT1 = eval(subs(TT1,{q1,q2,q3,q4,q5,q6},{q1_real(1),q1_real(2),q1_real(3),q1_real(4),q1_real(5),q1_real(6)}));  %第一组关节角
PP1 = TT1 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P1
pp1 = PP1(1:3,:);

TT2 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT2 = (subs(TT2,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT2 = eval(subs(TT2,{q1,q2,q3,q4,q5,q6},{q2_real(1),q2_real(2),q2_real(3),q2_real(4),q2_real(5),q2_real(6)}));  %第二组关节角
PP2 = TT2 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P2
pp2 = PP2(1:3,:);

TT3 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT3 = (subs(TT3,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT3 = eval(subs(TT3,{q1,q2,q3,q4,q5,q6},{q3_real(1),q3_real(2),q3_real(3),q3_real(4),q3_real(5),q3_real(6)}));  %第三组关节角
PP3 = TT3 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P3
pp3 = PP3(1:3,:);

TT4 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT4 = (subs(TT4,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT4 = eval(subs(TT4,{q1,q2,q3,q4,q5,q6},{q4_real(1),q4_real(2),q4_real(3),q4_real(4),q4_real(5),q4_real(6)}));  %第四组关节角
PP4 = TT4 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P4
pp4 = PP4(1:3,:);

TT5 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT5 = (subs(TT5,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT5 = eval(subs(TT5,{q1,q2,q3,q4,q5,q6},{q5_real(1),q5_real(2),q5_real(3),q5_real(4),q5_real(5),q5_real(6)}));  %第五组关节角
PP5 = TT5 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P5
pp5 = PP5(1:3,:);

TT6 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT6 = (subs(TT6,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT6 = eval(subs(TT6,{q1,q2,q3,q4,q5,q6},{q6_real(1),q6_real(2),q6_real(3),q6_real(4),q6_real(5),q6_real(6)}));  %第六组关节角
PP6 = TT6 * [0;0;0;1];       %实际DH参数下的基座坐标系下的坐标P6
pp6 = PP6(1:3,:);

TT7 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT7 = (subs(TT7,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT7 = eval(subs(TT7,{q1,q2,q3,q4,q5,q6},{q7_real(1),q7_real(2),q7_real(3),q7_real(4),q7_real(5),q7_real(6)}));  %第七组关节角
PP7 = TT7 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P7
pp7 = PP7(1:3,:);

TT8 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT8 = (subs(TT8,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT8 = eval(subs(TT8,{q1,q2,q3,q4,q5,q6},{q8_real(1),q8_real(2),q8_real(3),q8_real(4),q8_real(5),q8_real(6)}));  %第八组关节角
PP8 = TT8 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P8
pp8 = PP8(1:3,:);

TT9 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_true,a2_true,a3_true,a4_true,a5_true,a6_true}));
TT9 = (subs(TT9,{d1,d2,d3,d4,d5,d6},{d1_true,d2_true,d3_true,d4_true,d5_true,d6_true}));
TT9 = eval(subs(TT9,{q1,q2,q3,q4,q5,q6},{q9_real(1),q9_real(2),q9_real(3),q9_real(4),q9_real(5),q9_real(6)}));  %第九组关节角
PP9 = TT9 * [0;0;0;1];      %实际DH参数下的基座坐标系下的坐标P9
pp9 = PP9(1:3,:);
disp('实际坐标产生完毕')

%最小二乘法迭代DH参数
for i=1:1:5
    disp(i)
    a1_N=a1_N+delta(1,i);
    a2_N=a2_N+delta(2,i);
    a3_N=a3_N+delta(3,i);
    a4_N=a4_N+delta(4,i);
    a5_N=a5_N+delta(5,i);
    a6_N=a6_N+delta(6,i);
    d1_N=d1_N+delta(7,i);
    d2_N=d2_N+delta(8,i);
    d3_N=d3_N+delta(9,i);
    d4_N=d4_N+delta(10,i);
    d5_N=d5_N+delta(11,i);
    d6_N=d6_N+delta(12,i);
    %带入第一组角度 
    T1 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T1 = (subs(T1,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T1 = eval(subs(T1,{q1,q2,q3,q4,q5,q6},{q1_real(1),q1_real(2),q1_real(3),q1_real(4),q1_real(5),q1_real(6)}));       %将参数a,d,q带入T和L，并计算xyz位姿误差dP 4*4矩阵
    P1 = T1*[0;0;0;1];                     %得理论坐标
    P1_theory = P1(1:3,:);                 
    p1_true = pp1;                         %实际坐标pp1由DemarcateAssist的程序产生实际DH参数下的坐标
    L1 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q1_real(1),q1_real(2),q1_real(3),q1_real(4),q1_real(5),q1_real(6)})));
    B1=p1_true-P1_theory;  %1*3

    %带入第二组角度 
    T2 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T2 = (subs(T2,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T2 = eval(subs(T2,{q1,q2,q3,q4,q5,q6},{q2_real(1),q2_real(2),q2_real(3),q2_real(4),q2_real(5),q2_real(6)}));                
    P2 = T2*[0;0;0;1] ;           %得理论坐标
    P2_theory = P2(1:3,:);        
    p2_true = pp2;     %实际坐标pp2
    L2 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q2_real(1),q2_real(2),q2_real(3),q2_real(4),q2_real(5),q2_real(6)})));
    B2 = p2_true-P2_theory;

    %带入第三组角度
    T3 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T3 = (subs(T3,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T3 = eval(subs(T3,{q1,q2,q3,q4,q5,q6},{q3_real(1),q3_real(2),q3_real(3),q3_real(4),q3_real(5),q3_real(6)}));               
    P3 = T3*[0;0;0;1] ;                %得理论坐标
    P3_theory = P3(1:3,:);        
    p3_true = pp3;                      %实际坐标pp3
    L3 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q3_real(1),q3_real(2),q3_real(3),q3_real(4),q3_real(5),q3_real(6)})));
    B3 = p3_true-P3_theory;

    %带入第四组角度
    T4 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T4 = (subs(T4,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T4 = eval(subs(T4,{q1,q2,q3,q4,q5,q6},{q4_real(1),q4_real(2),q4_real(3),q4_real(4),q4_real(5),q4_real(6)}));                
    P4=T4*[0;0;0;1] ;              %得理论坐标
    P4_theory=P4(1:3,:);        
    p4_true=pp4;                    %实际坐标pp4  
    L4 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q4_real(1),q4_real(2),q4_real(3),q4_real(4),q4_real(5),q4_real(6)})));
    B4=p4_true-P4_theory;

    %带入第五组角度
    T5 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T5 = (subs(T5,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T5 = eval(subs(T5,{q1,q2,q3,q4,q5,q6},{q5_real(1),q5_real(2),q5_real(3),q5_real(4),q5_real(5),q5_real(6)}));                
    P5 = T5*[0;0;0;1] ;             %得理论坐标
    P5_theory = P5(1:3,:);        
    p5_true = pp5;        %实际坐标pp5   
    L5 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q5_real(1),q5_real(2),q5_real(3),q5_real(4),q5_real(5),q5_real(6)})));
    B5 = p5_true-P5_theory;

    %带入第六组角度
    T6 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T6 = (subs(T6,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T6 = eval(subs(T6,{q1,q2,q3,q4,q5,q6},{q6_real(1),q6_real(2),q6_real(3),q6_real(4),q6_real(5),q6_real(6)}));                
    P6 = T6*[0;0;0;1] ;             %得理论坐标
    P6_theory = P6(1:3,:);        
    p6_true = pp6;        %实际坐标pp6      
    L6 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q6_real(1),q6_real(2),q6_real(3),q6_real(4),q6_real(5),q6_real(6)})));
    B6 = p6_true-P6_theory;

    %带入第七组角度
    T7 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T7 = (subs(T7,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T7 = eval(subs(T7,{q1,q2,q3,q4,q5,q6},{q7_real(1),q7_real(2),q7_real(3),q7_real(4),q7_real(5),q7_real(6)}));                 
    P7 = T7*[0;0;0;1] ;           %得理论坐标
    P7_theory = P7(1:3,:);        
    p7_true = pp7;        %实际坐标pp7
    L7 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q7_real(1),q7_real(2),q7_real(3),q7_real(4),q7_real(5),q7_real(6)})));
    B7 = p7_true-P7_theory;

    %带入第八组角度
    T8 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T8 = (subs(T8,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T8 = eval(subs(T8,{q1,q2,q3,q4,q5,q6},{q8_real(1),q8_real(2),q8_real(3),q8_real(4),q8_real(5),q8_real(6)}));               
    P8 = T8*[0;0;0;1] ;           %得理论坐标
    P8_theory = P8(1:3,:);        
    p8_true = pp8;        %实际坐标pp8 
    L8 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q8_real(1),q8_real(2),q8_real(3),q8_real(4),q8_real(5),q8_real(6)})));
    B8 = p8_true-P8_theory;

    %带入第九组角度
    T9 = (subs(T,{a1,a2,a3,a4,a5,a6},{a1_N,a2_N,a3_N,a4_N,a5_N,a6_N}));
    T9 = (subs(T9,{d1,d2,d3,d4,d5,d6},{d1_N,d2_N,d3_N,d4_N,d5_N,d6_N}));
    T9 = eval(subs(T9,{q1,q2,q3,q4,q5,q6},{q9_real(1),q9_real(2),q9_real(3),q9_real(4),q9_real(5),q9_real(6)}));               
    P9 = T9*[0;0;0;1] ;           %得理论坐标
    P9_theory = P9(1:3,:);        
    p9_true = pp9;        %实际坐标pp9  
    L9 = eval((subs(L,{q1,q2,q3,q4,q5,q6},{q9_real(1),q9_real(2),q9_real(3),q9_real(4),q9_real(5),q9_real(6)})));
    B9 = p9_true-P9_theory;
    
    Q=[B1;B2;B3;B4;B5;B6;B7;B8;B9];  %B:3m*1  = 27*1  B1:3*1
    LL=[L1;L2;L3;L4;L5;L6;L7;L8;L9];  %L:3m*12 = 27*12 L1:3*12
%     disp('求解完成')
    delta(:,i+1) = pinv(LL'*LL)*LL'*Q;    %误差系数

end

disp('修正后的DH参数为：')
a_N = [a1_N;a2_N;a3_N;a4_N;a5_N;a6_N]
d_N = [d1_N;d2_N;d3_N;d4_N;d5_N;d6_N]