% Initial estimate of spec locations and surface normals using checkerboard
% estimate of the characterization-time camera position (to be refined with
% a geometric correction hereafter).
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/';
P.chardir = chardir;
P.leftRightSweep = [chardir 'verBarSweep/'];
P.upDownSweep = [chardir 'horBarSweep/'];
tStart = tic;

%% Get checkerboard points (image and world) and get homography
detector = vision.calibration.monocular.CheckerboardDetector();
fileName = [chardir 'checkerboards/A_glitterPosition.JPG'];
if ~exist([chardir 'imagePoints.mat'], "file")
    [imagePoints, ~] = detectPatternPoints(detector, fileName);
    save([chardir 'imagePoints'], "imagePoints");
    boardSize = detector.BoardSize;
    save([chardir 'boardSize'], "boardSize");
else
    imagePoints = matfile([chardir 'imagePoints']).imagePoints;
    boardSize = matfile([chardir 'boardSize']).boardSize;
    detector.BoardSize = boardSize;
end
squareSize = 18.25;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);
tform = estimateGeometricTransform(imagePoints, worldPoints, 'projective');
tformPath = [chardir 'tform.mat'];
save(tformPath,"tform");
P.tform = tformPath;

%% Detect specs:    find specs that sparkled during a sweep of light
fprintf("Detecting spec centroids...\n");
[P.imageCentroids, P.canonicalCentroids] = detectSpecs(P, chardir);
fprintf("%u specs detected after %f minutes\n", size(matfile(P.canonicalCentroids).canonicalCentroids,1), toc(tStart)/60);
%{
%% Max brightnesses:    find the peak brightness for each spec
fprintf("Finding spec max brightnesses...\n");
[P.maxBrightness] = maxBrightnesses(P, chardir);
fprintf("Brightnesses found after %f minutes\n", toc(tStart)/60);
%}

%% Get gaussians:   fit gaussians to the brightness distributions of
%                   specs across the lighting positions
fprintf("Fitting Gaussians to specs' brightness distributions...\n");
[P.means, P.imageCentroids, P.canonicalCentroids] = getGaussians(P, chardir);
fprintf("Gaussians fit after %f minutes\n", toc(tStart)/60);

%% Compute normals: using the known lighting positions and spec
%                   positions, compute the spec surface normals
fprintf("Computing spec surface normals...\n");
P = getMar4charPaths();
GLIT_TO_MON_PLANES = 424;
GLIT_TO_MON_EDGES_X = 178;
GLIT_TO_MON_EDGES_Y = 88.8 + 77.1 + 8*18.25;
charM = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,GLIT_TO_MON_EDGES_Y);
passedCamPos = matfile([chardir 'camPosSkew.mat']).camPos;
optionalName = 'spec_normals.mat';
P.specNormals = computeNormals(P, chardir, charM, passedCamPos, optionalName);
fprintf("Spec surface normals found after %f minutes\n", toc(tStart)/60);

% Display the final set of paths
disp(P);