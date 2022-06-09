% Given paths.mat file with updated paths to necessary images,
% this script runs a complete 'glitter characterization'.
% That is, it estimates the position of specs of glitter and their
% surface normals.
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

% Find transform:   compute homography from images to canonical
%                   coordinate system
T = saveTransform(P);

% Calibrate camera: find the camera's position in the canonical
%                   coordinate system using checkerboards
cam = calibrateCamera(P);

% Detect specs:     find specs that sparkled during a sweep of light
[imageCentroids, canonicalCentroids] = detectSpecs(P);

% Get gaussians:    fit gaussians to the brightness distributions of
%                   specs across the lighting positions
means = getGaussians(P);

% Compute normals:  using the known lighting positions and spec
%                   positions, compute the spec surface normals
specNormals = computeNormals(P);

