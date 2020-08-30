function[q] = Inverse_kinematics_POE(q0,Td)
%此程序是逆运动学数值求解,采用的方法是基于物体雅可比矩阵的牛顿-拉夫森法
%初始化：已知目标矩阵Td，初始角度估计值q0，设定i=0
%如果初始估计值与真实值没有足够接近，则迭代过程可能不收敛
%解出来的角度与初始角度有关
theta = q0;
for i = 1:1000
   disp(i)
   T_sb = Forward_kinematics(theta);
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
   %如果满足设定的误差最小值则停止循环
   if errorw < 0.00001 || errorv <0.00001
       q = theta;
       break
   end
   %不满足条件则修改theta的值
   %计算当前角度的物体雅克比矩阵
   J = Jacoby(theta);
   theta = theta + pinv(J)*Vb;
   for j = 1:6
       while theta(j) < -pi
           theta(j) = theta(j) + pi;
       end
       while theta(j) > pi
           theta(j) = theta(j) - pi;
       end
   end
end

end