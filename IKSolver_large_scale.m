%以下是关于逆运动学求解并对角度误差分析的程序
function[] = IKSolver_large_scale(delta)
tic
%可能用到的m为第几个关节角1-6   delta为加的角度误差值 0.1 or 0.2
qd = [0.5;0.5;0.5;0.5;0.5;0.5];    %初始位置
q0 = [0;0;0;0;0;0];             %初始位置去逼近
qd2 = [1;2;3;4;5;6];          %末端位置点

Td = FKSolver(qd);     %产生一个初始位姿Td
Td2 = FKSolver(qd2);   %产生一个末端位姿Td2
point = 30;     %点数
m = 1:1:point;  
%需要到达的目标点      两个位置点之间的距离
xdistance = Td2(13)-Td(13);
ydistance = Td2(14)-Td(14);
zdistance = Td2(15)-Td(15);
qd_delta1 = q0; 
qd1= zeros(6,point);
q_delta = zeros(6,point);   %预分配内存
xevery_point = xdistance/point;
yevery_point = ydistance/point;
zevery_point = zdistance/point;

for  n = 1:1:point     %直线点的求解  
    disp(n)
    qd1(:,n) = IKSolver(qd_delta1,Td);   %对初始的点进行求关节角 作为第一列  --- 第一次求解完成  qd_delta1 = q0
    if(qd1(:,n)==-1)
        q_delta(:,n)=[-0.1;-0.1;-0.1;-0.1;-0.1;-0.1];  %这个点求解不出来或者不在工作空间范围内则曲线中这些点表示-0.1
        continue
    end
    qdd= qd1(:,n); 
    qdd(1) = qdd(1)+delta;   %在第一个关节角上加上误差 delta 
    qdd(2) = qdd(2)+delta;   %在第二个关节角上加上误差 delta    
    qdd(3) = qdd(3)+delta;   %在第三个关节角上加上误差 delta    
    qdd(4) = qdd(4)+delta;   %在第四个关节角上加上误差 delta 
    qdd(5) = qdd(5)+delta;   %在第五个关节角上加上误差 delta    
%     qdd(6) = qdd(6)+delta;   %在某六个关节角上加上误差 delta 
    
    Tdd = FKSolver(qdd);   %误差点的位置姿态
    qd_delta = IKSolver(qd1(:,n), Tdd);  %求加上角度误差后的点的关节角  --  第二次求解完成  qd1(:,n) = q0
    if(qd_delta==-1)
        q_delta(:,n)=[-0.1;-0.1;-0.1;-0.1;-0.1;-0.1];  %这个点求解不出来或者不在工作空间范围内则曲线中这些点表示-0.1
        continue
    end
    q_delta(:,n) = qd_delta - qd1(:,n);      %与初始点的关节角误差
    qd_delta1 = qd_delta;
    Td(13) = Td(13)+xevery_point;   %每个点的xyz偏差距离
    Td(14) = Td(14)+yevery_point;
    Td(15) = Td(15)+zevery_point;
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



