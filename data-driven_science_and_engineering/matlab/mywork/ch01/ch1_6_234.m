clear all, close all, clc

load allFaces.mat



% We use the first 36 people for training data
traningFaces = faces(:,1:sum(nfaces(1:36)));
avgFace = mean(traningFaces, 2);

% Compute eigenfaces on mean-subtracted training data
X = traningFaces - avgFace*ones(1, size(traningFaces,2));
%X = traningFaces;
[U, S, V] = svd(X, 'econ');


subplot(2, 2, 1); imagesc(reshape(avgFace,n,m)); title('avg face');
subplot(2, 2, 2); imagesc(reshape(U(:,1), n, m)); title('1st. eigface');
subplot(2, 2, 3); imagesc(reshape(U(:,2), n, m)); title('2st. eigface');
subplot(2, 2, 4); imagesc(reshape(U(:,3), n, m)); title('3st. eigface');

%% Now show eigenface reconstruction of image that was omitted from test set
testFace = faces(:, 1+sum(nfaces(1:36)));
figure;
imagesc(reshape(testFace,n,m))

figure;
testFaceMS = testFace - avgFace;
count = 1;
for r = [25 50 100 200 400 800 1600]
    reconFace = avgFace + (U(:, 1:r) * (U(:, 1:r)'*testFaceMS));
    subplot(2, 4, count); imagesc(reshape(reconFace,n,m));
    count = count + 1;
end


%% Project person 2 and 7 onto PC5 and PC6
P1num = 2;
P2num = 7;

P1 = faces(:, 1+sum(nfaces(1:P1num-1)):sum(nfaces(1:P1num)));
P2 = faces(:, 1+sum(nfaces(1:P2num-1)):sum(nfaces(1:P2num)));

P1 = P1 - avgFace*ones(1,size(P1, 2));
P2 = P2 - avgFace*ones(1,size(P2, 2));

PCAModes = [5 6]; % Project onto PCA modes 5 and 6
PCACoordsP1 = U(:, PCAModes)'*P1;
PCACoordsP2 = U(:, PCAModes)'*P2;

figure;
plot(PCACoordsP1(1,:), PCACoordsP1(2,:), 'kd'); hold on;
plot(PCACoordsP2(1,:), PCACoordsP2(2,:), 'r^');



