% Visualize the lighting means
means = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/lightingmeans.mat').means;
figure;
hold on;
r = randperm(size(means,2));
plot(means(1,r(1:3000)),means(2,r(1:3000)),'rx');
title('monitor location that made each spec sparkle most')

% Visualize the image spec locations
specLocs = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/image_centroids.mat').imageCentroids;
figure;
hold on;
plot(specLocs(r(1:3000), 1),specLocs(r(1:3000), 2),'rx');
title('spec locations in image coordinates');

% Visualize the image spec locations
specLocs = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/canonical_centroids.mat').canonicalCentroids;
figure;
hold on;
plot(specLocs(r(1:3000), 1),specLocs(r(1:3000), 2),'rx');
title('spec locations in glitter coordinates');