%% do a checkerboard camera calibration
% set path to directory with checkerboards
% one of the files in this directory should be called onGlitterPlane.JPG
P.checkerboardIms = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/checkerboards/';


% fiducial marker points
allPts = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/16ptsJuly25.mat']).arr;
pin = allPts(1,:);
pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
pin = double([pinx' piny']);
fiducialMarkerPoints = pin;


% calibrate camera
calibrateCamera(P, fiducialMarkerPoints);

