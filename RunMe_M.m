clc;clear;close all
%   RunMe_M.m
%   一个简单的脚本实现Scara机器人的正逆运动学计算和动态演示。
%   author：ManQi
%   e-mail：zehuadu@126.com
%% 初始化
addpath(genpath(pwd));
sim_ang = [];
sim_angd = [];
sim_angdd = [];
%% 调用机器人工具箱进行机器人构建
% 机械臂参数
D(1) = 80;
D(2) = 50; % 大臂
D(3) = 0;
D(4) = 40; % 小臂
D(5) = 0;
D(6) = 0;

%             theta d      a    alpha sigma
L(1) = Link([ 0     D(1)   D(2)   0     0],  'mdh', 1);
L(2) = Link([ 0     0      D(4)   0     0],  'mdh', 1);
L(3) = Link([ 0     0      0      0     1],  'mdh', 1);
L(4) = Link([ 0     85     0      0     0],  'mdh', 1); % 该连杆中d无意义(实为0), 只用作辅助作图

% 运动范围
L(3).qlim = [0 -80];
L(1).qlim = [-170 170]*pi/180;
L(2).qlim = [-170 170]*pi/180;
L(4).qlim = [-170 170]*pi/180;

% 质量
L(1).m = 9.29;
L(2).m = 5.01;
L(3).m = 4.25;
L(4).m = 1.08;

% 电机惯量矩
L(1).Jm = 0.953;
L(2).Jm = 2.193;
L(3).Jm = 0.782;
L(4).Jm = 0.106;

% 传动比
L(1).G = 1;
L(2).G = 1;
L(3).G = 1;
L(4).G = 1;

% 连杆惯量矩
L(1).I = [0.276   0.255   0.071   0   0   0];
L(2).I = [0.108   0.018   0.100   0   0   0];
L(3).I = [2.51    2.51    0.006   0   0   0 ];
L(4).I = [0.002   0.001   0.001   0   0   0 ];

% 相对坐标系的质心位置
L(1).r = [0    0.0175 -0.1105];
L(2).r = [0   -1.054  0];
L(3).r = [0    0      -6.447];
L(4).r = [0    0.092  -0.054];

% qz = [0 0 0 0];

scara = SerialLink(L, 'name', 'scara');
scara.plotopt = {'workspace', [-160 160 -100 100 -12 200]};
scara.model3d = 'scara';
%% 指定起始和终止位置, IK进行笛卡尔空间到关节空间的变换
% 初始位姿
init_ang = [0 0 0 0];
iflags = 0;
if init_ang(1) < pi/2
    iflags = 1;
end
%% 执行
% 用户输入加工点并执行
model_motion = input('请输入运动模式, 0表示运动学, 1表示动力学:  ');
disp('请输入加工点表格名称, 如:''test.xlsx''(包含单引号)');
disp('数据格式, 如:');
disp(' |-----------------------------------|');
disp(' |   90   |   0    |   80   |   0    |');
disp(' |   50   |   50   |   50   |   pi/2 |');
disp(' |   x    |  y     |  z     |  theta |');
disp(' |-----------------------------------|');
file_name = input('文件名: ');
% 读入加工文件
pro_point = xlsread(file_name);
pro_point = [pro_point; [90 0 80 0]];
num_point = length(pro_point);

subplot(3,2,[1 3]);
scara.plot(init_ang);

% 遍历到达所有加工点
for item = 1:num_point
    x = pro_point(item, 1);
    y = pro_point(item, 2);
    z = pro_point(item, 3);
    c = pro_point(item, 4);
    joint = ScaraIK( x, y, z, c, iflags, D );
    % 轨迹规划
    start_ang = scara.getpos();
    targ_ang = joint;
    step=40;
    [q,qd,qdd] = jtraj(start_ang, targ_ang, step);
    tau = scara.rne(q,qd,qdd);
    subplot(3,2,2);
    temp = [q(:,1)*180/pi, q(:,2)*180/pi, q(:,3), q(:,4)*180/pi];
    plot(temp);
    title('期望的关节角度');
    hleg1 = legend('\theta_1', '\theta_2', '\theta_3', '\theta_4','Orientation','horizontal');
    set(hleg1, 'Position', [.10,.94,.4,.05]);
    subplot(3,2,4);
    temp = [qd(:,1)*180/pi, qd(:,2)*180/pi, qd(:,3), qd(:,4)*180/pi];
    plot(temp);
    title('期望的关节角速度');
    subplot(3,2,5);
    temp = [qdd(:,1)*180/pi, qdd(:,2)*180/pi, qdd(:,3), qdd(:,4)*180/pi];
    plot(temp);
    title('期望的关节角加速度');
    subplot(3,2,6);
    plot(tau);
    title('期望的关节扭矩');
    sim_time = 1;
    sim_ang.time = linspace(0,sim_time,step)';
    sim_ang.signals.values = q;
    sim_angd.time = linspace(0,sim_time,step)';
    sim_angd.signals.values = qd;
    sim_angdd.time = linspace(0,sim_time,step)';
    sim_angdd.signals.values = qdd;
    subplot(3,2,[1 3]);
    if model_motion == 0
        for i = 1:4:length(q)
        [ x, y, z, c ] = ScaraFK( q(i,:), D );
        line('xdata', [x, x + 0.001],...
             'ydata', [y, y + 0.001],...
             'zdata', [z, z + 0.001],...
             'color','g',...
             'linewidth',4);
        end
        scara.plot(q);
    else
        sim('scara_sim');
    end
end