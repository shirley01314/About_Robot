%以下是关于逆运动学求解并对角度误差分析的程序
function[] = IKSolver_lineanalyse(delta)
tic
%m为第几个关节角1-6   delta为加的角度误差值 0.1 or 0.2
qd = [0.5;0.5;0.5;0.5;0.5;0.5];    %只用做建立位置用
q0 = [0;0;0;0;0;0]; %初始位置去逼近

%程序正式开始 --------------
Td = FKSolver(qd);     %产生一个初始位姿Td
m = 1:1:28;
distance = -0.03;
qd_delta1 = q0; 
qd1= zeros(6,28);
q_delta = zeros(6,28);   %预分配内存

for  n = 1:1:28     %附近点为直线  分别在各点上的第一，二，三，四，五，六个关节角上加上误差0.1°
    disp(n)
    Td(13) = Td(13)+distance;   %得到偏差位置Td'
    Td(14) = Td(14)+distance;
    Td(15) = Td(15)+distance;
    qd1(:,n) = IKSolver(qd_delta,Td);   %对最开始的点进行求关节角 作为第一列  --- 第一次求解完成
    qdd= qd1(:,n); 
    qdd(1) = qdd(1)+delta;   %在某一个关节角上加上误差 delta 
%     qdd(2) = qdd(2)+0.2;   %在某一个关节角上加上误差 delta    
%     qdd(3) = qdd(3)+0.2;   %在某一个关节角上加上误差 delta    
    
    Tdd = FKSolver(qdd);   %误差点的位置姿态
    qd_delta = IKSolver(qd_delta, Tdd);  %求加上角度误差后的点的关节角  --  第二次求解完成
    q_delta(:,n) = qd_delta - qd1(:,n);      %与初始点的关节角误差
    
    Td(13) = Td(13)-distance;   %回到初始点
    Td(14) = Td(14)-distance;
    Td(15) = Td(15)-distance;
    
    distance = -0.03+n*0.002;
    qd_delta1 = qd_delta;
end

 h = figure(1);
set(h,'NumberTitle','off','name','在第n个关节角上加上0.1rad后各关节的误差值')

subplot(2,3,1)
plot(m, q_delta(1,:),'r*');
xlabel('点数');
ylabel('误差值');
axis square
grid on;
title('第1关节角误差曲线','FontName','黑体','FontSize',12)

subplot(2,3,2)
plot(m, q_delta(2,:),'r*');
xlabel('点数');
ylabel('误差值');
axis square
grid on;
title('第2关节角误差曲线','FontName','黑体','FontSize',12)

subplot(2,3,3)
plot(m, q_delta(3,:),'r*');
xlabel('点数');
ylabel('误差值');
axis square
grid on;
title('第3关节角误差曲线','FontName','黑体','FontSize',12)

subplot(2,3,4)
plot(m, q_delta(4,:),'r*');
xlabel('点数');
ylabel('误差值');
axis square
grid on;
title('第4关节角误差曲线','FontName','黑体','FontSize',12)

subplot(2,3,5)
plot(m, q_delta(5,:),'r*');
xlabel('点数');
ylabel('误差值');
axis square
grid on;
title('第5关节角误差曲线','FontName','黑体','FontSize',12)

subplot(2,3,6)
plot(m, q_delta(6,:),'r*');
xlabel('点数');
ylabel('误差值');
axis square
grid on;
title('第6关节角误差曲线','FontName','黑体','FontSize',12)
toc
end



