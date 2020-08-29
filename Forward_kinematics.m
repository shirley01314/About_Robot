function[ Tout ] = Forward_kinematics(q)
%输入六个关节角为实际转动的关节角，CRP的零位关节角为[0;pi/2;0;0;pi/2;0]
%此程序为CRP正运动学计算，根据实际机械臂建立坐标系
%连杆长度  相对于前一个坐标系的建立
Tx = [0, -0.1573, -0.0165, -0.2975, -0.3545, 0.0820];   
Ty = [0, 0.0653, 0.0464, -0.0519, 0.0440, -0.0145];     
Tz = [0, 0, 0.6136, 0.1671, -0.0437, -0.5149];          
%连杆转角
alpha = [0, pi/2, 0, -pi/2, -pi/2, pi/2];
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
%末端执行器相对于惯性系0的变换矩阵
Tout = T(:,:,1) * T(:,:,2) * T(:,:,3) * T(:,:,4) * T(:,:,5) * T(:,:,6);

end




