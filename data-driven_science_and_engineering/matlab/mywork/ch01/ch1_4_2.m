clear all, close all, clc

load hald;
A = ingredients;
b = heat;

[U, S, V] = svd(A, 'econ');
x = V*inv(S)*U'*b;


plot(b, 'k' , 'LineWidth', 2);  hold on
plot(A*x, 'r-o',  'LineWidth', 1, 'MarkerSize',2);
legend('Heat data', 'Regression')

%% Altnetive1 (regress)
x = regress(b,A)

%% Altnetive 2(pinv)
x = pinv(A)*b

%% Altnetive 3(Normal equation)
x = inv(A'*A)*A'*b
