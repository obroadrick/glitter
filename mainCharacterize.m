% Given paths.mat file with updated paths to necessary images,
% this script runs a complete 'glitter characterization'.
% That is, it estimates the position of specs of glitter and their
% surface normals.
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

%% Fiducial marker points:  
% open and interprety addy's homography points file then pass 
% the points to the future programs
allPts = matfile(P.characterizationPoints).arr;
pin = allPts(1,:);
pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
fiducialMarkerPoints = double([pinx' piny']);

%% Find transform:  compute homography from images to canonical
%                   coordinate system
tStart = tic;
fprintf("Computing homography...\n");
P.tform = saveTransform(P, fiducialMarkerPoints);
fprintf("Homography computed after %f minutes\n", toc(tStart)/60);

%% Calibrate camera: find the camera's position in the canonical
%                    coordinate system using checkerboards
fprintf("Calibrating camera...\n");
P.camParams = calibrateCamera(P, fiducialMarkerPoints);
fprintf("Camera calibrated after %f minutes\n", toc(tStart)/60);

%% Detect specs:    find specs that sparkled during a sweep of light
fprintf("Detecting spec centroids...\n");
[P.imageCentroids, P.canonicalCentroids] = detectSpecs(P);
fprintf("%u specs detected after %f minutes\n", size(matfile(P.canonicalCentroids).canonicalCentroids,1), toc(tStart)/60);

%% Max brightnesses:    find the peak brightness for each spec
fprintf("Finding spec max brightnesses...\n");
[P.maxBrightness] = maxBrightnesses(P);
fprintf("Brightnesses found after %f minutes\n", toc(tStart)/60);

%% Get gaussians:   fit gaussians to the brightness distributions of
%                   specs across the lighting positions
fprintf("Fitting Gaussians to specs' brightness distributions...\n");
[P.means, P.imageCentroids, P.canonicalCentroids] = getGaussians(P);
fprintf("Gaussians fit after %f minutes\n", toc(tStart)/60);

%% Compute normals: using the known lighting positions and spec
%                   positions, compute the spec surface normals
fprintf("Computing spec surface normals...\n");
P.specNormals = computeNormals(P);
fprintf("Spec surface normals found after %f minutes\n", toc(tStart)/60);

% Save the updated paths struct
save([P.data 'paths'], "P");