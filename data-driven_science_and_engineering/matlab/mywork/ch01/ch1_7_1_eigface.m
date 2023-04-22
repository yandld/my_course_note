clear all, close all, clc

load 'allFaces.mat'
trainingData = faces(:, 1:sum(nfaces(1:37)));
avgface = mean(trainingData, 2);
imagesc(reshape(avgface, n, m));
X = trainingData - avgface*ones(1, size(trainingData, 2));
[U, S, V] = svd(X, 'econ');

%%
sigs = diag(S);
beta = size(X,2) / size(X,1);
thresh = optimal_SVHT_coef(beta, 0) * median(sigs);

figure;
semilogy(sigs, '-ok');
grid on
hold on

xlim([0 length(sigs)]);
ylim([1 10^6]);
semilogy(sigs(sigs>thresh), 'bo');

rvals = [200 766 2000];
sigsold = diag(S);
semilogy(rvals, sigsold(rvals), 'co');


for r = rvals;
    figure;
    imagesc(reshape(U(:,r),n,m)); colormap gray
    axis off
end









