clear all, close all, clc

x=-5:0.1:5; y=-6:0.1:6; t=0:0.1:10*pi;
[X,Y,T]=meshgrid(x,y,t);

%% generate data
f1 =exp(-(X.^2+0.5*Y.^2)).*(cos(2*T));
f2 =  (sech(X).*tanh(X).*exp(-0.2*Y.^2)).*sin(T);
A = f1+f2;

%% plot feature
% for j=1:length(t)
%   surf(x,y,A(:,:,j)), shading interp, caxis([-1 1]), drawnow
% end

%% flat data
nx = length(x); ny=length(y);
for j=1:length(t)
    Af(:,j) = reshape( A(:,:,j), nx*ny, 1);
end

%% 
model = parafac(A,2);
[A1, A2, A3] = fac2let(model);
subplot(3,1,1), plot(y,A1,'Linewidth',[2])
subplot(3,1,2), plot(x,A2,'Linewidth',[2])
subplot(3,1,3), plot(t,A3,'Linewidth',[2])





