%���³�������֤CRPʾ�����ϵ�λ�����ݺ��Ӿ���׽ϵͳ��õ���������֮������
%���е��ĩ������ϵΪC ��е�ۻ�����ϵΪA  �Ӿ���׽ϵͳ������ϵΪB
%�Ի�е�۵���λ���Լ��ڻ�е�۵��ĸ����������ȡ����в���
%��������ϵ1�½���
%��һ�������ȡΪ��е�۵���λ��
%ĩ��λ��C�������ϵA֮��ı任��ϵΪ
Tca1 = [1  0  0  718.084;
     0  1 0  0.054;
     0  0  1  284.846;
     0  0  0    1];
 %�Ӿ���׽��λ����ϵB����е�ۻ�����ϵA֮��Ĺ�ϵ ��2-3mm����� 
Tba = [1  0  0 471.387;
     0  1  0  645.382;
     0  0  1  -179.297;
     0  0  0     1]; 
 %ĩ��λ�����Ӿ���׽ϵͳ�µ�����λ��
Tcb1 = [1 0 0 259.435;
         0 1 0 -639.145;
         0 0 1 464.4638;
         0 0 0  1];
%֮���λ�����
error1 = Tca1 -  Tcb1*Tba

%�ڻ�е�۵�����ȡ�����������  �ڶ������� +x��+y������ȡ��
Tca2 = [1  0  0  547.148;
     0  1 0  513.575;
     0  0  1  231.921;
     0  0  0    1];
 %�Ӿ���׽��λ����ϵB����е�ۻ�����ϵA֮��Ĺ�ϵ ��2-3mm����� 
Tba = [1  0  0 471.387;
     0  1  0  645.382;
     0  0  1  -179.297;
     0  0  0     1]; 
 %ĩ��λ�����Ӿ���׽ϵͳ�µ�����λ��
Tcb2 = [1 0 0 82.68275;
         0 1 0 -127.156;
         0 0 1 409.5226;
         0 0 0  1];
error2 = Tca2 -  Tcb2*Tba

%�ڻ�е�۵�����ȡ�����������  ���������� -x��+y������ȡ�� 
Tca3 = [1  0  0  -733.727;
     0  1 0  513.624;
     0  0  1  231.936;
     0  0  0    1];
 %�Ӿ���׽��λ����ϵB����е�ۻ�����ϵA֮��Ĺ�ϵ ��2-3mm����� 
Tba = [1  0  0 471.387;
     0  1  0  645.382;
     0  0  1  -179.297;
     0  0  0     1]; 
 %ĩ��λ�����Ӿ���׽ϵͳ�µ�����λ��
Tcb3 = [1 0 0 -1199.49;
         0 1 0 -131.278;
         0 0 1 420.748;
         0 0 0  1];
error3 = Tca3 -  Tcb3*Tba   

%�ڻ�е�۵�����ȡ�����������  ���ĸ����� -x��-y������ȡ��
Tca4 = [1  0  0  -263.142;
     0  1 0  -856.093;
     0  0  1  231.908;
     0  0  0    1];
 %�Ӿ���׽��λ����ϵB����е�ۻ�����ϵA֮��Ĺ�ϵ ��2-3mm����� 
Tba = [1  0  0 471.387;
     0  1  0  645.382;
     0  0  1  -179.297;
     0  0  0     1]; 
 %ĩ��λ�����Ӿ���׽ϵͳ�µ�����λ��
Tcb4 = [1 0 0 -723.009;
         0 1 0 -1500.92;
         0 0 1 429.8514;
         0 0 0  1];
error4 = Tca4 -  Tcb4*Tba   %���ĵ�����Դ󣬲���Խ����ä������׽����ȷ������Խ��

%�ڻ�е�۵�����ȡ�����������  ��������� +x��-y������ȡ��
Tca5 = [1  0  0  703.217;
     0  1 0  -554.638;
     0  0  1  231.902;
     0  0  0    1];
 %�Ӿ���׽��λ����ϵB����е�ۻ�����ϵA֮��Ĺ�ϵ ��2-3mm����� 
Tba = [1  0  0 471.387;
     0  1  0  645.382;
     0  0  1  -179.297;
     0  0  0     1]; 
 %ĩ��λ�����Ӿ���׽ϵͳ�µ�����λ��
Tcb5 = [1 0 0 241.9256;
         0 1 0 -1197.72;
         0 0 1 419.9679;
         0 0 0  1];
error5 = Tca5 -  Tcb5*Tba













 
 
 