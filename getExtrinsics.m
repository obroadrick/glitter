% given the save parameters and parameter errors from the camera
% calibration app, this script computes and saves the camera extrinsics
% relative to the glitter coordinate system

%% get inputs
% experiment path: (a directory under which all the relevant information
% for this particular experiment is stored)
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep18characterization(new-1)/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep19characterization(new-2)/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/oct17characterization/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/oct25_nikonz7_35mm/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_far/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/wideAngle/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/wideAngleCardboard/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/iphoneXR/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/iphoneXR2/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/iphone/';
%expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/testingMatlab/odds/';

%index=2;
numcases = 10;
for index = 1:1
%expir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/' num2str(index) '/'];
expir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(index) '/'];
expdir = expir;

skew = true;
% get P
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
% get camParams
%camParams = matfile(P.camParams).camParams;
%camParams = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/camParams_07_25_2022.mat').camParams;%july25test

if ~skew
    camParams = matfile([expir 'camParams.mat']).camParams;
    camParamsErrors = matfile([expir 'camParamsErrors.mat']).camParamsErrors;
else
    camParams = matfile([expir 'camParamsSkew.mat']).camParams;
    camParamsErrors = matfile([expir 'camParamsErrorsSkew.mat']).camParamsErrors;
end
% get reference image path
%imPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/checkerboards/onGlitterPlane.JPG';
%imPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep1characterization/checkerboards/onGlitterPlane.JPG';
%impath = [expir 'onGlitterPlane' num2str(index) '.JPG'];
impath = [expir 'A_onGlitterPlane' num2str(index) '.JPG'];
imPath = impath;
%imPath = [expir 'homography.JPG'];
%imPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/checkerboards/onGlitterPlane.JPG';
% get fiducial marker points
%allPts = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/16ptsJuly25.mat']).arr;
%allPts = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/16ptsAug18.mat').arr;
if ~isfile([expdir '16pts.mat'])
    % if the 16pts for this experiment haven't already been found, then
    % find them now using Addy's script
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', impath, [expdir '16pts.mat']);
    system(cmd);
end
%{
allPts = matfile([expir '16Pts.mat']).arr;
pin = allPts(1,:);
pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
pin = double([pinx' piny']);
%}
pin = loadPoints([expir '16Pts.mat'], true);

%% run computation
% run findCamPos which computes the camPos and camRot for these inputs
[t, R, terr, Rerr, Rvec] = findCamPos(P, camParams, camParamsErrors, impath, pin);
camPosErr = terr;
camRotErr = Rerr;

%% save results
camPos = t;
camRot = R;
%{
P.camPos = [P.data 'camPos_' datestr(now, 'mm_dd_yyyy')];
P.camRot = [P.data 'camRot_' datestr(now, 'mm_dd_yyyy')];
save([P.data 'camPos_' datestr(now, 'mm_dd_yyyy')], "camPos");
save([P.data 'camRot_' datestr(now, 'mm_dd_yyyy')], "camRot");
%}
% also store the pos and rot in the experiment directory;
%TODO actually check when it is with and without skew estimate
if ~skew
    save([expir 'camPos'], "camPos");
    save([expir 'camRot'], "camRot");
    save([expir 'camPosErr'], "camPosErr");
    save([expir 'camRotErr'], "camRotErr");
else
    save([expir 'camPosSkew'], "camPos");
    save([expir 'camRotSkew'], "camRot");
    save([expir 'camPosErrSkew'], "camPosErr");
    save([expir 'camRotErrSkew'], "camRotErr");
end
end% end big for loop