%以下程序是六关节机械臂的结构参数定义及其牛顿欧拉的方法调用
syms q1 q2 q3 q4 q5 q6 dq1 dq2 dq3 dq4 dq5 dq6 ddq1 ddq2 ddq3 ddq4 ddq5 ddq6
%连杆长度a0 = 0 ; a1 = 0.3581; a2 = 0.6137; a3 = 0.3575; a4 = 0.3664; a5 = 0.12
dh_parms = [0,0,0,q1;
            -pi/2,0.3581,0,q2;
            0,0.6137,0,q3-pi/2;
            -pi/2,0.3575,0,q4;
            pi/2,0.3664,0,q5;
            -pi/2,0.12,0,q6;
            0,0,0,0];
mass_center = [0.0467, -0.0104, 0.1100;
               -0.0155, -0.2895, -0.0268;
               0.1366, 0.1311, 0.0806;
               0.0350, 0.0044, 0.1474;
               3.9663e-06, -0.0019, 0.0540;
               0, 0, 0.05];
mass  = [17.9513, 3.5484, 7.3201, 3.8682, 0.7287, 1]; %六个连杆的质量]
inertia_1 = [ 0.5354,0.0131,-0.2059;
            0.0131,0.7118,0.0404;
            -0.2059,0.0404,0.5010];
inertia_2 = [0.5044, -0.0164, -0.0021;
    -0.0164, 0.0144, -0.0304;
    -0.0021, -0.0304, 0.5091];
inertia_3 = [0.2601, -0.1844, -0.0883;
    -0.1844, 0.2780, -0.0850;
    -0.0883, -0.0850, 0.3979];
inertia_4 = [0.1544, -0.0001, -0.0143;
    -0.0001, 0.1527, -0.0051;
    -0.0143, -0.0051, 0.0224];    
inertia_5 = [0.0055, 0, 0;
    0, 0.0040, -0.0015;
    0, -0.0015, 0.0028]; 
inertia_6 = [0.0042, 0, 0;
        0, 0.005, 0;
        0, 0, 0.02];      
    
inertia_tensor(:,:,1) = inertia_1;
inertia_tensor(:,:,2) = inertia_2;
inertia_tensor(:,:,3) = inertia_3;
inertia_tensor(:,:,4) = inertia_4;
inertia_tensor(:,:,5) = inertia_5;
inertia_tensor(:,:,6) = inertia_6;
f_ext = [0,0,0;
         0,0,0;
         0,0,0;
         0,0,0;
         0,0,0;
         0,0,0];
torque = NewtonEulerDynamics(dh_parms, mass, mass_center, inertia_tensor, f_ext);  

torque1 = vpa(torque(1),4);
torque2 = vpa(torque(2),4);
torque3 = vpa(torque(3),4);
torque4 = vpa(torque(4),4);
torque5 = vpa(torque(5),4);
torque6 = vpa(torque(6),4); 


