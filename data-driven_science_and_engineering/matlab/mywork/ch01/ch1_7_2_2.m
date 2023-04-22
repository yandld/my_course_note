clear all, close all, clc

n = 1000;
X = zeros(n,n);
X(n/4:3*n/4,n/4:3*n/4) = 1;
nAngles = 14;  % sweep through 12 different angles, from 0:4:44

[U, S, V] = svd(X);
subplot(1,2,1), imagesc(X), hold on;
subplot(1,2,2), semilogy(diag(S), '-o'); grid on; hold on;

Xrot = X;
for j=2:nAngles
    Y = imrotate(X, (j-1)*4, 'bicubic');
    startind = floor((size(Y,1)-n)/2);
    Xrot1 = Y(startind:startind+n-1, startind:startind+n-1);
    Xrot(Xrot1 > 0.5) = j;
    
    [U, S, V] = svd(Y);
    subplot(1,2,1), imagesc(Xrot);
    subplot(1,2,2), semilogy(diag(S), '-o');
end





