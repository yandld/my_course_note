clear all, close all, clc

load ovariancancer.mat;
[U, S, V] = svd(obs, 'econ');

figure; hold on
for i=1:size(obs,1)
    x = V(:,1)'*obs(i,:)'; % V(:,1)' 就是最大的那个基
    y = V(:,2)'*obs(i,:)';
    z = V(:,3)'*obs(i,:)';
    if(grp{i}=='Cancer')
        plot3(x,y,z,'rx','LineWidth',2);
    else
        plot3(x,y,z,'bo','LineWidth',2);
    end
end
view(85,25), grid on, set(gca,'FontSize',13)