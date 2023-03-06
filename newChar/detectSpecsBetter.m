% detect specs for characterizing given max image
function [C] = detectSpecsBetter(impath)
    % code pulled form visualizeDetectedSpecs.m where I figured out this better
    % way of detecting them
    im = imread(impath);
    % get local maxima
    X = imregionalmax(rgb2gray(im));
    threshold = 30;
    X = X & rgb2gray(im)>threshold;
    % find centroids of regions that are AND of previous two conditions
    R = regionprops(X,'Centroid','PixelIdxList','PixelList','Area');
    for rx = 1:size(R,1)
        ptsAlsoThreshold(rx,:) = R(rx).Centroid;
    end
    % clean up: keep only ones that are well in the squares after homography
    P = getMar4charPaths();
    tform = matfile(P.tform).tform;
    C = ptsAlsoThreshold;
    out = transformPointsForward(tform, [C(:,1) C(:,2)]);
    C_canonical = [out(:,1) out(:,2) zeros(size(out,1),1)];
    idx = cleanPoints(C_canonical, 18.25, .15);
    C_canonical_keep = C_canonical(idx,:);
    C_clean = C(idx,:);
    
    % get rid of egregiously close-together points
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
end