#!/usr/bin/python3       添加这行 可在linux终端下执行python代码的文件  直接./example1.py
# -*- coding: UTF-8 -*-    在python2中可支持中文字符

import sympy
import sympybotics

# 搭建CRP前六轴DH参数 1.1 和 1.2分别是a1和a2的长度 四个参数分别为 alpha, a, d, theta    0   0.1703  0.6155  0.3451  0.3599  0.5216
rbtdef = sympybotics.RobotDef('SixJoint', [('0', 0.00, 0.0, 'q1'),
                                           ('-pi/2', 0.1703, 0.0, 'q'),
                                           ('0', 0.6155, 0.0, 'q-pi/2'),
                                           ('-pi/2', 0.3451, 0.0, 'q'),
                                           ('pi/2', 0.3599, 0.0, 'q'),
                                           ('-pi/2', 0.5216, 0.0,'q')],
                                            dh_convention ='modified')

# rbtdef.frictionmodel = {'Coulomb', 'viscous'} 加上摩擦系数和阻尼系数
# 设定重力加速度的值
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])
# 显示动力学全参数
print(rbtdef.dynparms())
# 构建机器人模型
rbt = sympybotics.RobotDynCode(rbtdef,verbose=True)

tau_str = sympybotics.robotcodegen.robot_code_to_func('C',rbt.invdyn_code,'tau_out','tau',rbtdef)

print(tau_str)
# 向量Mregress中的元素分别表示各关节力矩值在标准参数XB下的系数 torque1=Mregress[0:29]  torque2=Mregress[30:59]  torque3=Mregress[60:89]
# 向量Mregress是标准参数30*1系数 而不是简化后的最小参数集 且未加库伦摩擦和粘性摩擦
# 计算并显示动力学模型的回归矩阵
Mregress = sympybotics.robotcodegen.robot_code_to_func('C',rbt.H_code,'Mregress','Mregress',rbtdef)
print(Mregress)
# 计算动力学模型的最小参数集
rbt.calc_base_parms()
print(rbt.dyn.baseparms)
