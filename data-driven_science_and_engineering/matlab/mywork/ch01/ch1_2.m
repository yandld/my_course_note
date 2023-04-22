clear all, close all, clc


A= imread('dog.jpg');
X = double(rgb2gray(A));
nx  = size(X, 1); ny = size(X,2);


[U, S, V] = svd(X);

plotind = 1;
subplot(2,2,plotind);
imagesc(X); axis off, colormap gray 
title("Original");
plotind = plotind + 1;
    
for r = [5 20 100]
    Xapprox = U(:,1:r) * S(1:r, 1:r) * V(:,1:r)';
    subplot(2,2,plotind); imagesc(Xapprox), axis off, colormap gray
    title(['r=',num2str(r,'%d'),', ',num2str(100*r*(nx+ny)/(nx*ny),'%2.2f'),'% storage']);
    plotind = plotind + 1;
end

figure, subplot(1,2,1);
semilogy(diag(S), 'k', 'LineWidth', 1.2), grid on
xlabel('r')
ylabel('Singular value, \sigma_r')
xlim([-50 1550])
subplot(1,2,2)
plot(cumsum(diag(S)) / sum(diag(S)), 'k', 'LineWidth', 1.2), grid on
xlabel('r')
ylabel('Cumulative Energy')
xlim([-50 1550]); ylim([0 1.1])

