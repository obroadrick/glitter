% given the save parameters and parameter errors from the camera
% calibration app, this script computes and saves the camera extrinsics
% relative to the glitter coordinate system

%% get inputs
% experiment path: (a directory under which all the relevant information
% for this particular experiment is stored)
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/';
expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/';
% get P
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
% get camParams
%camParams = matfile(P.camParams).camParams;
%camParams = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/camParams_07_25_2022.mat').camParams;%july25test
camParams = matfile([expir 'camParams.mat']).camParams;%aug18test
% get reference image path
imPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/checkerboards/onGlitterPlane.JPG';
%imPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/checkerboards/onGlitterPlane.JPG';
% get fiducial marker points
%allPts = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/16ptsJuly25.mat']).arr;
allPts = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/16ptsAug18.mat').arr;
pin = allPts(1,:);
pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
pin = double([pinx' piny']);

%% run computation
% run findCamPos which computes the camPos and camRot for these inputs
[t, R] = findCamPos(P, camParams, imPath, pin);

%% save results
camPos = t;
camRot = R;
P.camPos = [P.data 'camPos_' datestr(now, 'mm_dd_yyyy')];
P.camRot = [P.data 'camRot_' datestr(now, 'mm_dd_yyyy')];
save([P.data 'camPos_' datestr(now, 'mm_dd_yyyy')], "camPos");
save([P.data 'camRot_' datestr(now, 'mm_dd_yyyy')], "camRot");
% also store the pos and rot in the experiment directory;
save([expir 'camPos'], "camPos");
save([expir 'camRot'], "camRot");