function [ x, y, z, c ] = ScaraFK( joint, D )
% [ x, y, z, c ] = ScaraFK( joint, D )
% Scara��е�۵������˶�ѧ����
%   input��
%      joint�� �ؽڿռ���Ϣ
%      D��     �ṹ����
%   output��
%      x, y, z, c: �ѿ����ռ���Ϣ
%   author��ManQi
%   e-mail��zehuadu@126.com

D1 = D(1);
D2 = D(2);
D3 = D(3);
D4 = D(4);
D5 = D(5);
D6 = D(6);


% /* convert joint angles to radians for sin() and cos() */
    a0 = joint(1);

    a1 = joint(2);

    a3 = joint(4);

% /* convert angles into world coords */
    a1 = a1 + a0;

    a3 = a3 + a1;

    x = D2*cos(a0) + D4*cos(a1) + D6*cos(a3);

    y = D2*sin(a0) + D4*sin(a1) + D6*sin(a3);

    z = D1 + D3 + joint(3) - D5;

    c = a3;

end

