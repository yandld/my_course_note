clc;
clear;
close all;


R = 0.01; %量测噪声
Q = 0.1*eye(2); %过程噪声
u = -2; %控制输入
dt = 0.5;

X = [0 5]'; %状态量
P = [0.01 0; 0 1]; %过程方差阵
y = pi / 6; %第一次量测更新量

S = 20;
D = 40;
F =  [1 dt; 0 1];

%  predict
X =F*X + [0 dt]'*u;
P = F*P*F' + Q;
fprintf("进行一步预测后状态:\n");
X
fprintf("进行一步预测后状态方差:\n");
P


% update hx and Jacc: H
hx = atan(S / (D - X(1)));
H = [S / ((D - X(1))^(2) + S^(2)), 0];

% update
K = P*H'*(H*P*H' +R )^(-1);
X = X + K*(y - hx);
P = (eye(2) - K*H)*P;

fprintf("量测更新后系统状态\n");
X

fprintf("量测更新后系统方差(不确定性)变小了:\n");
P
