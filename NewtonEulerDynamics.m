function torque_list = NewtonEulerDynamics(dh_list, mass_list, mass_center_list, inertia_tensor_list, f_external)
% 输入参数：
% dh_list：机器人DH参数
% mass_list: 连杆的质量，单位kg
% mass_center_list：连杆质心在连杆坐标系下的位置，单位：m
% inertia_tensor_list：连杆关于质心坐标系的惯性张量，质心坐标系与连杆坐标系方位一致
% f_external：施加在末端连杆的外力和外力矩

% 输出参数：
% torque_list：用q,dq,ddq（关节位置、速度、加速度）表达的关节力矩
[rows, columns] = size(dh_list);
number_of_links = rows - 1;
if columns ~= 4
    error('wrong DH parameters!')
end

T = sym([]);
R = sym([]);
a = sym([]);
d = sym([]);
alpha = sym([]);
theta = sym([]);

for i = 1:rows
    % 定义关节位置，速度，加速度符号
    eval(['syms ','q',num2str(i),' real;']);
    eval(['syms ','dq',num2str(i),' real;']);
    eval(['syms ','ddq',num2str(i),' real;']);
    eval(['q(i)=','q',num2str(i),';']);
    eval(['dq(i)=','dq',num2str(i),';']);
    eval(['ddq(i)=','ddq',num2str(i),';']);
end

for i = 1:rows
    dh = dh_list(i,:);
    alpha(i) = dh(1);
    a(i) = dh(2);
    d(i) = dh(3);
    theta(i) = dh(4);
    if i == rows
        q(i) = 0;
    end
    T(:,:,i) = [cos(q(i)),            -sin(q(i)),           0,           a(i);
            sin(q(i))*cos(alpha(i)), cos(q(i))*cos(alpha(i)), -sin(alpha(i)), -sin(alpha(i))*d(i);
            sin(q(i))*sin(alpha(i)), cos(q(i))*sin(alpha(i)), cos(alpha(i)), cos(alpha(i))*d(i);
            0,                     0,                     0,          1];
    T = T(:,:,i);
    % 提取旋转矩阵并求逆
    R(:,:,i) = simplify(inv(T(1:3,1:3)));
    P(:,:,i) = T(1:3,4:4);
end

% 关节轴
z = [0,0,1]';
% 重力加速度符号
syms g real

% 外推 --->
disp('外推')  %0-5
for i = 0:number_of_links-1
    disp(i)
    if i == 0
        wi = [0,0,0]';
        dwi = [0,0,0]';
        dvi = [0, g, 0]';
    else
        wi = w(:,i);
        dwi = dw(:,i);
        dvi = dv(:,i);
    end
    w(:,:,i+1) = R(:,:,i+1)*wi + dq(i+1)*z;
    dw(:,:,i+1) = R(:,:,i+1)*dwi + cross(R(:,:,i+1)*wi,dq(i+1)*z) + ddq(i+1)*z;
    dv(:,:,i+1) = R(:,:,i+1)*(cross(dwi,P(:,:,i+1)) + cross(wi,cross(wi,P(:,:,i+1))) + dvi);
    dvc(:,:,i+1) = cross(dw(:,:,i+1),mass_center_list(i+1,:)')...
                    + cross(w(:,:,i+1),cross(w(:,:,i+1),mass_center_list(i+1,:)'))...
                    + dv(:,:,i+1);
    F(:,:,i+1) = mass_list(i+1)*dvc(:,:,i+1);
    N(:,:,i+1) = inertia_tensor_list(:,:,i+1)*dw(:,:,i+1) + cross(w(:,:,i+1),inertia_tensor_list(:,:,i+1)*w(:,:,i+1));
end

f = sym([]);
n = sym([]);

% 内推 <---
disp('内推') %6-1
for i = number_of_links:-1:1
    disp(i)
    if i == number_of_links
        f(:,:,i+1) = f_external(1,:)';
        n(:,:,i+1) = f_external(2,:)';
    end
    f(:,:,i) = R(:,:,i+1)\f(:,:,i+1) + F(:,:,i);
    f(:,:,i) = simplify(f(:,:,i));
    n(:,:,i) = N(:,:,i) + R(:,:,i+1)\n(:,:,i+1) + cross(mass_center_list(i,:)',F(:,:,i))...
                + cross(P(:,:,i+1),R(:,:,i+1)\f(:,:,i+1));
    n(:,:,i) = simplify(n(:,:,i));
    torque_list(i) = dot(n(:,:,i),z);
end
disp('递推完成')
torque_list = torque_list';
