function [ joint ] = ScaraIK( x, y, z, c, iflags, D )
% [ x, y, z, c ] = ScaraFK( joint, D )
% Scara机械臂的逆向运动学计算
%   input：
%      x, y, z, c： 笛卡尔空间信息
%      iflags：     滑动轴运动方向
%      D：     结构参数
%   output：
%      joint： 关节空间信息
%   author：ManQi
%   e-mail：zehuadu@126.com

a3 = c;
% /* center of end effector (correct for D(6)) */
xt = x - D(6) * cos(a3);
yt = y - D(6) * sin(a3);
% /* horizontal distance (squared) from end effector centerline
% to main column centerline */
rsq = xt*xt + yt*yt;
% /* joint 1 angle needed to make arm length match sqrt(rsq) */
cc = (rsq - D(2)*D(2) - D(4)*D(4)) / (2*D(2)*D(4));
if(cc < -1)
    cc = -1;
end
if(cc > 1)
    cc = 1;
end
q1 = acos(cc);
if (iflags)
    q1 = -q1;
end
% /* angle to end effector */
q0 = atan2(yt, xt);
% /* end effector coords in inner arm coord system */
xt = D(2) + D(4)*cos(q1);
yt = D(4)*sin(q1);
% /* inner arm angle */
q0 = q0 - atan2(yt, xt);
% /* q0 and q1 are still in radians. convert them to degrees */
% q0 = q0 * (180 / pi);
% q1 = q1 * (180 / pi);

joint(1) = q0;
joint(2) = q1;
% joint(3) = D(1) + D(3) - D(5) + z;
joint(3) = -D(1) - D(3) - D(5) + z;
joint(4) = c - ( q0 + q1);
end

