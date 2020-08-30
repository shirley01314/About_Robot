function[q] = Modern_Robotics_IKsolver61(q0,Td)
%现代机器人学例题6.1的逆运动学求解
theta = q0;
for i = 1:10
   disp(i)
    T_sb = Modern_Robotics_FKsolver61(theta);
    T_bd = inv(T_sb)*Td;  %以e为底
    Vb_frame = logm(T_bd);
    v = Vb_frame(1:3,4);
    w_frame = Vb_frame(1:3,1:3);
    %速度旋量Vb 维度6*1 
    Vb = [w_frame(6);w_frame(7);w_frame(2);v];
    xitew = Vb(1:3);
    xitev = Vb(4:6);
    errorw = norm(xitew)
    errorv = norm(xitev)
    if errorw < 0.001 || errorv <0.0001
       q = theta;
       break
    end 
    %如果满足设定的误差最小值则停止循环
    %不满足条件则修改theta的值
    %计算当前角度的物体雅克比矩阵
    J = Modern_Robotics_Jacobe61(theta);
    pinv(J)*Vb;
    theta = theta + pinv(J)*Vb;
    for j = 1:2
       while theta(j) < -pi
           theta(j) = theta(j) + pi;
       end
       while theta(j) > pi
           theta(j) = theta(j) - pi;
       end
    end
   theta*180/pi
end

end

