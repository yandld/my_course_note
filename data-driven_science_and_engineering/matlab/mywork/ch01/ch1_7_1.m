clear all, close all, clc

%%
t = (-3: 0.01: 3)';
Utrue = [cos(17*t).*exp(-t.^2) sin(11*t)];
Strue = [2 0; 0 0.5];
Vtrue = [sin(5*t).*exp(-t.^2) cos(13*t)];

X = Utrue*Strue*Vtrue';
subplot(2,2,1); imshow(X);
title("Orignal");

%%
sigma = 1;
Xnoisy = X + sigma*randn(size(X));
subplot(2,2,2); imshow(Xnoisy)
title("Noisy");

%%
[U, S, V] = svd(Xnoisy);
N = size(Xnoisy, 1);
cutoff = (4/sqrt(3)) * sqrt(N) * sigma;
r = max(find(diag(S) > cutoff));
Xclean = U(:,1:r) * S(1:r, 1:r) * V(:, 1:r)';
subplot(2,2,3); imshow(Xclean);
title("Hard Threshold");

%% 
cdS = cumsum(diag(S)) ./sum(diag(S)); % Cumlative energy
r90 = min(find(cdS > 0.9)); % Find r to capture 90% energy
X90 = U(:,1:r90) * S(1:r90, 1:r90) * V(:, 1:r90)';
subplot(2,2,4); imshow(X90);
title("90% cutoff");

%%
figure;
semilogy(diag(S), '-ok'), hold on, grid on;
semilogy(diag(S(1:r, 1:r)), 'or');
 



