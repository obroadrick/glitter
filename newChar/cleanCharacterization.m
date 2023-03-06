% Clean up the characterization by keeping only the specs have reasonable
% positions.

%% get current centroids and other relevant info
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/';
P.chardir = chardir;
P = getMar4charPaths();
GLIT_TO_MON_PLANES = 424;
GLIT_TO_MON_EDGES_X = 178;
GLIT_TO_MON_EDGES_Y = 88.8 + 77.1 + 8*18.25;
M = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,GLIT_TO_MON_EDGES_Y);
imageCentroids = matfile([chardir 'image_centroids']).imageCentroids;
canonicalCentroids = matfile([chardir 'canonical_centroids']).canonicalCentroids;

%% find the clean subset of the current centroids
d = M.CALIBRATION_SQUARE_SIZE;
buf = 0.15;
idx = cleanPoints(canonicalCentroids, d, buf);

%% visualize the chosen clean subset to confirm correctness
figure;
hold on;
r = randperm(size(canonicalCentroids,1));
n1 = 3000;
plot(canonicalCentroids(r(1:n1),1), canonicalCentroids(r(1:n1),2), 'rx');
n2 = 5000;
for ix = 1:n2
    if idx(r(ix)) == 1
        plot(canonicalCentroids(r(ix),1), canonicalCentroids(r(ix),2), 'bx');
    end
end

%% save results
%save(imageCentroidsPath, "imageCentroids");
%save(canonicalCentroidsPath, "canonicalCentroids");

%% functions
function [idx] = cleanPoints(C, d, buf)
    idx = zeros(1,size(C,1));
    % throw out points that are not within the middle 1-2buf interval of
    % a square
    for ix = 1:size(C,1)
        loc = C(ix,1:2);
        p = (loc(1) / d) - floor(loc(1) / d);
        q = (loc(2) / d) - floor(loc(2) / d);
        if p > buf && p < 1-buf && q > buf && q < 1-buf
            idx(ix) = 1;
        end
        num = floor(loc(1) / d) + floor(loc(2) / d);
        % but then if this is not even one of the black squares, it is set
        % to zero regardless
        if mod(num,2) == 1
            idx(ix) = 0;
        end
        % finally also check if this is just off the board
        x = floor(loc(1) / d);
        y = floor(loc(2) / d);
        if x < -1 || x > 12 || y < -1 || y > 9
            idx(ix) = 0;
        end
    end
end
