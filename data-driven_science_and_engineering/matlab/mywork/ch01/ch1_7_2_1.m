clear all, close all, clc

n = 1000;
X = zeros(n, n);
X(n/4:3*n/4,n/4:3:n/4) = 1;
subplot(1,2,1); imshow(X);

Y = imrotate(X,10,'bicubic');  % rotate 10 degrees
Y = Y - Y(1,1);
nY = size(Y,1);

startind = floor((nY-n)/2);
Xrot = Y(startind:startind+n-1, startind:startind+n-1);
subplot(1,2,2); imshow(Xrot);

figure;
[U,S,V] = svd(X);     % SVD well-aligned square
subplot(1,2,1); semilogy(diag(S),'-ko');
[U,S,V] = svd(Xrot);  % SVD rotated square
subplot(1,2,2); semilogy(diag(S),'-ko');




