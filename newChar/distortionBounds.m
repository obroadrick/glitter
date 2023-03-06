% need to understand the distortion encoded by radial distortion
% coefficients k1 and k2 to settle on a reasonable range of those values to
% consider

% i will do so by computing, in a big ol' grid, how far a corner point
% eg (0,0) gets distorted for each of the coefficient pairs k1,k2 in a big
% range and then find the bounds that distort the corner by at most some
% proportion (.25 maybe?) of the image width/height

% find lens distortion and camera translation from glitter to camera 
% using a known light source, a picture of sparkling glitter, and a 
% known glitter characterization (spec positions and surface normals)

rng(314159);
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
M = matfile(P.measurements).M;

% read in image
%impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
%im = rgb2gray(imread(impath));

%% grid search over k1,k2
p = [0,0];
r1 = .5;
s1 = .002;
r2 = .5;
s2 = .005;
k1s = -r1:s1:r1;
k2s = -r2:s2:r2;
distorted = [];
for k1ix=1:size(k1s,2)
    for k2ix=1:size(k2s,2)
        k1 = k1s(k1ix);
        k2 = k2s(k2ix);
        pu = undistortRadially(p, k1, k2);
        distorted(k1ix,k2ix,:) = pu;
    end
end
%% show
figure;
plot(p(:,1),p(:,2),'cx','MarkerSize',12,'LineWidth', 1.5); hold on;
for k1ix=1:size(k1s,2)
    for k2ix=1:size(k2s,2)
        k1 = k1s(k1ix);
        k2 = k2s(k2ix);
        text(reshape(distorted(k1ix,k2ix,1),1,1),...
            reshape(distorted(k1ix,k2ix,2),1,1),...
            [num2str(k1) ' ' num2str(k2)]);
        hold on;
    end
end
%% generate the grid search space and then show it
% we will let both span from -.5 to .5 and then subject them to the
% constraint |2k1 + 4k2| <= a = .25 (.25 for an example)
% a is the proportion of its radius that any point can 'move' in or out
a = .25;
stepsize = .05;
overall_lb = -.5;
overall_ub = .5;
k2s = linspace(overall_lb,overall_ub,(overall_ub-overall_lb)/stepsize);
kspace = [];
numPairs = 0;
for k2ix=1:size(k2s,2)
    k2 = k2s(k2ix);
    curlb = (-a-4*k2)/2;
    curub = (a-4*k2)/2;
    lb = max(overall_lb, curlb);
    ub = min(overall_ub, curub);
    if lb >= ub
        continue
    end
    curk1s = linspace(lb, ub, ceil((ub-lb)/stepsize));
    for k1ix=1:size(curk1s,2)
        numPairs = numPairs + 1;
        kspace(numPairs,:) = [curk1s(k1ix) k2];
    end
end

%% plot/show kspace
plot(kspace(:,1), kspace(:,2), 'r+', 'MarkerSize', 12);
xlabel('k1');
ylabel('k2');
title('distortion coefficients grid search space');
