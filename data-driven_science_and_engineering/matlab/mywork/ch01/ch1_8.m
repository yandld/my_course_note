clear all, close all, clc

A =imread('jupiter.jpg');
X = double(rgb2gray(A));

tic;
[U, S, V] = svd(X, 'econ');
toc;

r = 50;  % Target rank
q = 1;     % Power iterations
p = 5;     % Oversampling parameter

tic;
[rU, rS, rV] = rsvd(X, r, q, p);
toc;

XSVD = U(:,1:r)*S(1:r,1:r)*V(:,1:r)';
errSVD = norm(X- XSVD, 2) / norm(X,2);
XrSVD = rU(:,1:r)*rS(1:r,1:r)*rV(:,1:r)';
errrSVD = norm(X- XrSVD, 2) / norm(X,2);

imagesc(XSVD);


subplot(1,3,1);
imagesc(X);
subplot(1,3,2);
imagesc(XSVD);
subplot(1,3,3);
imagesc(XrSVD);

% clear all, close all, clc
% 
% m=1000;
% Omega = randn(m, 10);
% A = (Omega.')*Omega;
% pcolor(abs(A)), colorbar
% 





