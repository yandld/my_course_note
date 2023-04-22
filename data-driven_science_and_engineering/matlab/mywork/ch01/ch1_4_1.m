clear all, close all, clc

x = 3;
a = [-2:0.25:2]';
b = a*x + 1*randn(size(a));
hold on, plot(a, b, 'rx');

%%
[U,S,V] = svd(a, 'econ');
xtilde = V*inv(S)*U'*b;
plot(a, xtilde*a, 'b--');

xtitle1 = V*inv(S)*U'*b
xtitle2 = pinv(a)*b
xtitle3 = regress(b, a)
xtitle4 = inv(a'*a)*a'*b