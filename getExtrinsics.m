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
%overallExpDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/';
%overallExpDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/';
overallExpDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/';
numcases = 10;

for index = 1:10
expir = [overallExpDir num2str(index) '/'];
expdir = expir;
skew = false;
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

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
if ~isfile([expdir '16pts.mat'])
    % if the 16pts for this experiment haven't already been found, then
    % find them now using Addy's script
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', impath, [expdir '16pts.mat']);
    system(cmd);
end
pin = loadPoints([expir '16Pts.mat'], true);

%% 
% run findCamPos which computes the camPos and camRot for these inputs
[t, R, terr, Rerr, Rvec] = findCamPos(P, camParams, camParamsErrors, impath, pin);
camPosErr = terr;
camRotErr = Rerr;
camPos = t;
camRot = R;

%% 
% save results for this sub-experiment
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

% Track the overall set of checkerboard results for this experiment so that
% we can save that at the end also
omega = undoRodrigues(camRot);
fx = camParams.Intrinsics.FocalLength(1);
fy = camParams.Intrinsics.FocalLength(2);
cx = camParams.Intrinsics.PrincipalPoint(1);
cy = camParams.Intrinsics.PrincipalPoint(2);
s = camParams.Intrinsics.Skew;
rotAndIntrinsicsCheckerboards = [camPos omega(1) omega(2) omega(3) fx fy cx cy s];

allData(index,:) = rotAndIntrinsicsCheckerboards';

end% end big for loop


% Save the overall results for this set of checkerboard images for this
% experiment for easy/quick access in future
checkerResults = allData;
save([overallExpDir 'checkerResults.mat'], "checkerResults");
