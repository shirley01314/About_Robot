y2 = z2;
% figure
% x和P的初值是可以随便设的   P的初值不能为0  Q = 0.01;  R = 1; P = 1  x0 = -30
% kalman_filter(data,Q,R,x0,P0)
result = kalman_filter(y2,1e-6,4e-4,y2(1),1);
i = 1:length(y2);
% plot(i,result,'r',i,y2,'b');
figure
subplot(2,1,1)
plot(y2,'b');
subplot(2,1,2)
plot(i,result,'r');


