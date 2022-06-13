% Given paths.mat file with updated paths to necessary images,
% this script runs a complete 'glitter characterization'.
% That is, it estimates the position of specs of glitter and their
% surface normals.
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

%% Find transform:  compute homography from images to canonical
%                   coordinate system
tStart = tic;
fprintf("Computing homography...\n");
P.tform = saveTransform(P);
fprintf("Homography computed after %f minutes\n", toc(tStart)/60);

%% Skip camera-calibration for now as it is done... %%%%%%%%%%%%%%%%%
%% Calibrate camera:find the camera's position in the canonical
%                   coordinate system using checkerboards
%fprintf("Calibrating camera...\n");
%P.camPos = calibrateCamera(P);
%fprintf("Camera calibrated after %f minutes\n", toc(tStart)/60);

%% Detect specs:    find specs that sparkled during a sweep of light
fprintf("Detecting spec centroids...\n");
[P.imageCentroids, P.canonicalCentroids] = detectSpecs(P);
fprintf("%u specs detected after %f minutes\n", size(canonicalCentroids,1), toc(tStart)/60);

%% Get gaussians:   fit gaussians to the brightness distributions of
%                   specs across the lighting positions
fprintf("Fitting Gaussians to specs' brightness distributions...\n");
P.means = getGaussians(P);
fprintf("Gaussians fit after %f minutes\n", toc(tStart)/60);

%% Compute normals: using the known lighting positions and spec
%                   positions, compute the spec surface normals
fprintf("Computing spec surface normals...\n");
P.specNormals = computeNormals(P);
fprintf("Spec surface normals found after %f minutes\n", toc(tStart)/60);

% Save the updated paths struct
save([P.data 'paths'], "P");