%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%此程序在YHXCRP14_Trajectory.slx和SevenTrajectory.m程序运行之后再运行
%以用于得到轨迹规划末端轨迹的xyz方向上的误差
k = 100;  %比较的点数
errorx = zeros(1,k);
errory = zeros(1,k);
errorz = zeros(1,k);
Tx = zeros(1,k);
Ty = zeros(1,k);
Tz = zeros(1,k);
%末端轨迹

for i=1:1:100
    disp(i)
    Fq1 = q1.Data(1+10*(i-1));
    Fq2 = q2.Data(1+10*(i-1));
    Fq3 = q3.Data(1+10*(i-1));
    Fq4 = q4.Data(1+10*(i-1));
    Fq5 = q5.Data(1+10*(i-1));
    Fq6 = q6.Data(1+10*(i-1));
    Tout= YHXFKSolver([Fq1;Fq2;Fq3;Fq4;Fq5;Fq6]);
    Tx(:,i) = Tout(13);
    Ty(:,i) = Tout(14);
    Tz(:,i) = Tout(15);
    
    errorx(:,i) = Tx(:,i) - lx(:,i);  %实际位置 - 理论位置
    errory(:,i) = Ty(:,i) - ly(:,i);
    errorz(:,i) = Tz(:,i) - lz(:,i);
 
end
m = 1:1:k;
h = figure(1);
set(h,'NumberTitle','off','name','xyz方向上的实际路径和路径位移误差')

%实际得到xyz方向上的误差曲线
subplot(2,3,1)
plot(m, Tx,'LineWidth',1.5,'color','b');
xlabel('实际位移','FontName','黑体','FontSize',12);
ylabel('误差值','FontName','黑体','FontSize',12);
set(gca,'YLim',[min(Tx)-(max(Tx)-min(Tx))/10 max(Tx)+(max(Tx)-min(Tx))/10]);
axis square
grid on;
title('x方向上轨迹曲线','FontName','黑体','FontSize',12)
hold on

subplot(2,3,2)
plot(m, Ty,'LineWidth',1.5,'color','b');
xlabel('实际位移','FontName','黑体','FontSize',12);
ylabel('误差值','FontName','黑体','FontSize',12);
set(gca,'YLim',[min(Ty)-(max(Ty)-min(Ty))/10 max(Ty)+(max(Ty)-min(Ty))/10]);
axis square
grid on;
title('y方向上轨迹曲线','FontName','黑体','FontSize',12)
hold on

subplot(2,3,3)
plot(m, Tz,'LineWidth',1.5,'color','b');
xlabel('实际位移','FontName','黑体','FontSize',12);
ylabel('误差值','FontName','黑体','FontSize',12);
set(gca,'YLim',[min(Tz)-(max(Tz)-min(Tz))/10 max(Tz)+(max(Tz)-min(Tz))/10]);
axis square
grid on;
title('z方向上轨迹曲线','FontName','黑体','FontSize',12)
hold on

%实际xyz与理想xyz的轨迹误差曲线
subplot(2,3,4)
plot(m, errorx,'LineWidth',1.5,'color','b');
xlabel('位移误差','FontName','黑体','FontSize',12);
ylabel('误差值','FontName','黑体','FontSize',12);
set(gca,'YLim',[min(errorx)-(max(errorx)-min(errorx))/10 max(errorx)+(max(errorx)-min(errorx))/10]);
axis square
grid on;
title('x方向上误差曲线','FontName','黑体','FontSize',12)
hold on

subplot(2,3,5)
plot(m, errory,'LineWidth',1.5,'color','b');
xlabel('位移误差','FontName','黑体','FontSize',12);
ylabel('误差值','FontName','黑体','FontSize',12);
set(gca,'YLim',[min(errory)-(max(errory)-min(errory))/10 max(errory)+(max(errory)-min(errory))/10]);
axis square
grid on;
title('y方向上误差曲线','FontName','黑体','FontSize',12)
hold on

subplot(2,3,6)
plot(m, errorz,'LineWidth',1.5,'color','b');
xlabel('位移误差','FontName','黑体','FontSize',12);
ylabel('误差值','FontName','黑体','FontSize',12);
set(gca,'YLim',[min(errorz)-(max(errorz)-min(errorz))/10 max(errorz)+(max(errorz)-min(errorz))/10]);
axis square
grid on;
title('z方向上误差曲线','FontName','黑体','FontSize',12)


    