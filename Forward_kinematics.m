function[ Tout ] = Forward_kinematics(q)
%注：输入六个关节角为实际转动的关节角
%CRP的零位关节角为[0;pi/2;0;0;pi/2;0]
%此程序为CRP正运动学计算，根据实际机械臂建立坐标系
%连杆长度  相对于基坐标系在前一个关节坐标系下的建立
%搭建CRP前六轴DH参数 连杆长度  a0-a5  0  0.1702  0.6155  0.3451  0.3599  0.0904 末端执行器a6 = 0.4341 单位mm 
Tx = [0, 0.15725, -0.61360, -0.16714, 0.04373, 0.00621];   
Ty = [0, -0.06528, -0.01653, -0.29747, -0.04398, -0.09014];     
Tz = [0, 0, -0.04641, 0.05193, 0.35455, 0.00206];          
%连杆转角
alpha = [0, -pi/2, 0, pi/2, pi/2, -pi/2];
%关节角 
theta = [q(1);q(2)+pi/2;q(3);q(4);q(5)+pi/2;q(6)];
T = zeros(4,4,6);
%先绕x轴旋转再绕z轴旋转  相对于前一个坐标系
for i = 1:6  
    T(:,:,i) = [cos(theta(i)) -sin(theta(i)) 0 Tx(i);
                cos(alpha(i))*sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i)) Ty(i);
                sin(alpha(i))*sin(theta(i)) sin(alpha(i))*cos(theta(i)) cos(alpha(i)) Tz(i);
                0 0 0 1];
end
%第六关节法兰盘中心处相对于惯性系0的变换矩阵
T6 = T(:,:,1) * T(:,:,2) * T(:,:,3) * T(:,:,4) * T(:,:,5) * T(:,:,6);

%末端执行器相对于第六关节法兰盘中心处的变换矩阵
T76 = [1 0 0 -0.08824;
       0 1 0 0.01657;
       0 0 1 -0.42473;
       0 0 0 1];
   
Tout = T6    ;
%末端执行器相对于基坐标系0的变换矩阵
% Tout = T6 * T76;
end









