clear all, close all, clc

load housing.data

b = housing(:, 14);
A = housing(:,1:13);
A = [A ones(size(A,1), 1)];

subplot(1,2,1)
x = regress(b, A);
plot(b, 'k-o'); hold on
plot(A*x, 'r-o');

subplot(1,2,2)
[b sortind] = sort(housing(:,14));
plot(b, 'k-o')
hold on, plot(A(sortind, :)*x, 'r-o')


figure;
bar(x(1:13))