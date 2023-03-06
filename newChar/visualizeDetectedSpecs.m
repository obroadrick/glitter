% Visualize the specs that we're detecting and characterizing
% there are three categories of specs:
% red, detected only
% blue, detected and characterized succesfully (lighting mean found)
% green, detected, characterized, and within the bounds of the squares


% Visualize the image spec locations
specLocsDetected = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/image_centroids_detect_specs.mat').imageCentroids;
specLocsCharacterized = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/image_centroids.mat').imageCentroids;
specLocsCanonicalCharacterized = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/canonical_centroids.mat').canonicalCentroids;
idx = cleanPoints(specLocsCanonicalCharacterized, 18.25, .15);
specLocsCleaned = specLocsCharacterized(idx,:);

im = imread('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/maxImageLeftRight.jpg');

figure;
tiledlayout(1,2,'TileSpacing','tight','Padding','tight');
axes(1) = nexttile;
imagesc(im);
hold on;
set(gca,'YTickLabel',[], 'XTickLabel',[]);
%set(gca,'YDir','reverse');
%plot(specLocsDetected(:, 1),specLocsDetected(:, 2),'rx');
%plot(specLocsCharacterized(:, 1),specLocsCharacterized(:, 2),'bx');
plot(specLocsCleaned(:, 1),specLocsCleaned(:, 2),'gx','LineWidth',2,'MarkerSize',2);
title(sprintf('Old Detected Spec Locations (%d)',size(specLocsCleaned,1)));
xlabel('filter, threshold')
%{
%%
% play around with filters to see if more specs can be easily detected
figure;
tiledlayout(1,6,'TileSpacing','tight','Padding','tight');

axes(1) = nexttile; imagesc(im);set(gca,'YTickLabel',[], 'XTickLabel',[]);

d = 40; r1 = 1; r2 = 7;
F = fspecial('Gaussian',[d d],r1) - fspecial('Gaussian',[d d],r2);
axes(2) = nexttile; imagesc(imfilter(im, F)>20);set(gca,'YTickLabel',[], 'XTickLabel',[]);

d = 40; r1 = 3; r2 = 15;
F = fspecial('Gaussian',[d d],r1) - fspecial('Gaussian',[d d],r2);
axes(3) = nexttile; imagesc(imfilter(im, F)>20);set(gca,'YTickLabel',[], 'XTickLabel',[]);

axes(4) = nexttile;
X = imregionalmax(rgb2gray(im));
imagesc(X);
title('imregionalmax');
colormap(gray);
set(gca,'YTickLabel',[], 'XTickLabel',[]);

R = regionprops(X,'Centroid','PixelIdxList','PixelList','Area');
for rx = 1:size(R,1)
    pts(rx,:) = R(rx).Centroid;
end

axes(5) = nexttile;
imagesc(im);hold on;
plot(pts(:,1), pts(:,2),'rx');
title('using local maxima');
set(gca,'YTickLabel',[], 'XTickLabel',[]);

% could look at gradients
linkaxes(axes);
%}
%% more
% get local maxima
X = imregionalmax(rgb2gray(im));
threshold = 30;
% find centroids of regions that are AND of previous two conditions
R = regionprops(X,'Centroid','PixelIdxList','PixelList','Area');
for rx = 1:size(R,1)
    ptsJustRegionalMax(rx,:) = R(rx).Centroid;
end
X = X & rgb2gray(im)>threshold;
% find centroids of regions that are AND of previous two conditions
R = regionprops(X,'Centroid','PixelIdxList','PixelList','Area');
for rx = 1:size(R,1)
    ptsAlsoThreshold(rx,:) = R(rx).Centroid;
end
%%
% clean up: keep only ones that are well in the squares after homography
P = getMar4charPaths();
tform = matfile(P.tform).tform;
C = ptsAlsoThreshold;
out = transformPointsForward(tform, [C(:,1) C(:,2)]);
C_canonical = [out(:,1) out(:,2) zeros(size(out,1),1)];
idx = cleanPoints(C_canonical, 18.25, .15);
C_canonical_keep = C_canonical(idx,:);
C_clean = C(idx,:);

%% get rid of egregiously close-together points
% n^2 brute force way to do it... :/
points = C_clean;
d = 4;
for i = 1:size(points, 1)
    % Compute distances from current point to all other points
    distances = pdist2(points(i,:), points);
    
    % Find indices of points within distance d
    indices = find(distances <= d & distances > 0);
    
    % If there are other points within distance d, replace them with their average/midpoint
    if ~isempty(indices)
        % Compute the average/midpoint of all points within distance d
        avg_point = mean(points([i, indices],:), 1);
        
        % Replace the original points with the average/midpoint
        points([i, indices],:) = repmat(avg_point, length(indices)+1, 1);
    end
end
% do it again to account for little clusters of 3
for i = 1:size(points, 1)
    % Compute distances from current point to all other points
    distances = pdist2(points(i,:), points);
    
    % Find indices of points within distance d
    indices = find(distances <= d & distances > 0);
    
    % If there are other points within distance d, replace them with their average/midpoint
    if ~isempty(indices)
        % Compute the average/midpoint of all points within distance d
        avg_point = mean(points([i, indices],:), 1);
        
        % Replace the original points with the average/midpoint
        points([i, indices],:) = repmat(avg_point, length(indices)+1, 1);
    end
end
C = unique(points,'rows');
%%
% visualize
%{
axes(2) = nexttile;
imagesc(im);
hold on;
%plot(ptsJustRegionalMax(:,1), ptsJustRegionalMax(:,2),'rx');
%plot(ptsATlsoThreshold(:,1), ptsAlsoThreshold(:,2),'gx');
plot(C_clean(:,1), C_clean(:,2),'rx');
title('using local maxima AND above threshold');
set(gca,'YTickLabel',[], 'XTickLabel',[]);
%{
axes(2) = nexttile;
imagesc(imread('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/homography/2023-03-02T14,12,26homography-Glitter.JPG'));
%}
%}
axes(2) = nexttile;
imagesc(im);
hold on;
%plot(ptsJustRegionalMax(:,1), ptsJustRegionalMax(:,2),'rx');
%plot(ptsAlsoThreshold(:,1), ptsAlsoThreshold(:,2),'gx');
%title('find local maxima; apply intensity threshold; require minimum separation');
xlabel('local maxima, threshold, min separation')
plot(C(:,1), C(:,2),'gx','LineWidth',2,'MarkerSize',2);
title(sprintf('New Detected Spec Locations (%d)',size(C,1)));
set(gca,'YTickLabel',[], 'XTickLabel',[]);
linkaxes(axes);


