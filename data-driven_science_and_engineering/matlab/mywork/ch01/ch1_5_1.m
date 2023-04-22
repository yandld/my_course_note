clear all, close all, clc

xC  = [2; 1]; % center of data(mean)
sig = [2; .5]; % principal axes

theta = pi / 10;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

nPoints = 10000;
X = R*diag(sig)*randn(2, nPoints) + diag(xC)*ones(2,nPoints);
%scatter(X(1,:), X(2,:), 'k.', 'LineWidth', 2);


%%
Xavg = mean(X, 2);                      %Computer mean
B = X - Xavg*ones(1, nPoints);      % Mean-subtracted Data
C = B'*B / (nPoints-1);

[U, S, V] = svd(B / sqrt(nPoints), 'econ');


scatter(X(1,:), X(2,:), 'k.');hold on,  grid on

theta = (0: 0.01: 1)*2*pi;
Xstd = U*S*[cos(theta); sin(theta)]; % 1-std conf. interval
plot(Xavg(1)+Xstd(1,:),Xavg(2) + Xstd(2,:),'r-','LineWidth',1.5)
plot(Xavg(1)+2*Xstd(1,:),Xavg(2) + 2*Xstd(2,:),'r-','LineWidth',1.5)
plot(Xavg(1)+3*Xstd(1,:),Xavg(2) + 3*Xstd(2,:),'r-','LineWidth',1.5)

% Plot principal components U(:,1)S(1,1) and U(:,2)S(2,2)
plot([Xavg(1) Xavg(1)+U(1,1)*S(1,1)],[Xavg(2) Xavg(2)+U(2,1)*S(1,1)],'c-','LineWidth',2)
plot([Xavg(1) Xavg(1)+U(1,2)*S(2,2)],[Xavg(2) Xavg(2)+U(2,2)*S(2,2)],'c-','LineWidth',2)




