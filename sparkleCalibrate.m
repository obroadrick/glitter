% single image camera calibration with glitter

%% input: 
%  path to the single image
%  path to matfile with fiducial marker points
%  known point light source position
%  characterized sheet of glitter: spec locations, normals
%% output:
%  saves as matfile all the parameters estimated:
% intrinsic matrix, 
% rotation matrix, 
% rodrigues parameters,
% focal length x and y,
% skew,
% image center x and y,
% distortion paramters k1 and k2


%% set inputs

%  path to the single image
%impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july19characterization/circleOnMonitor/2022-07-19T13,54,52circle-calib-W1127-H574-S48.jpg';
impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/glitter/DSC_3113.JPG';
%impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/singleImageAug18.JPG';

% path to single image fiducial marker points
% get by running Addy's Python script on the single image:
allPts = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/16ptsJuly25.mat']).arr;
%allPts = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/16ptsAug18.mat']).arr;
pin = allPts(1,:);
pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
pin = double([pinx' piny']);
fiducialMarkerPoints = pin;

%  characterized sheet of glitter: spec locations, normals
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
M = matfile(P.measurements).M;
% note: in practice right now these P and M structs are just read in fresh
% in each of the estimate T and RK scripts below... so the characterization
% and relevant measurements are not set and passed in this script

%  known point light source position
%{
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];
%}
lightPos = [0 125-73 535];%july25nikonz7
%lightPos = [0 129-72.9 527];%aug18nikonz7

% estimate translation and distortion
% todo/future version

%% estimate translation
disp('Estimating camera position with sparkles...');
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin);
disp('Position estimate complete!');

%% first retrieve the checkerboard calibration information for comparison along the way
format shortG;%display numbers in more reasonable way
camParams = matfile(P.camParams).camParams;
camParamsErrors = matfile(P.camParamsErrors).camParamsErrors;
camPos = matfile(P.camPos).camPos;
camRot = matfile(P.camRot).camRot;
% rotAndIntrinsics = [omega1 omega2 omega3 fx fy cx cy s]
omega = undoRodrigues(camRot);
fx = camParams.Intrinsics.FocalLength(1);
fy = camParams.Intrinsics.FocalLength(2);
cx = camParams.Intrinsics.PrincipalPoint(1);
cy = camParams.Intrinsics.PrincipalPoint(2);
s = camParams.Intrinsics.Skew;
rotAndIntrinsicsCheckerboards = [omega(1) omega(2) omega(3) fx fy cx cy s];
disp('Estimate of rotation and intrinsics using checkerboards');
disp('[      omega1       omega2       omega3           fx           fy           cx           cy            s]');
disp(rotAndIntrinsicsCheckerboards);
% also show their error estimates each
fxe = camParamsErrors.IntrinsicsErrors.FocalLengthError(1);
fye = camParamsErrors.IntrinsicsErrors.FocalLengthError(2);
cxe = camParamsErrors.IntrinsicsErrors.PrincipalPointError(1);
cye = camParamsErrors.IntrinsicsErrors.PrincipalPointError(2);
se = camParamsErrors.IntrinsicsErrors.SkewError;
rotAndIntrinsicsCheckerboardsErrors = [-1 -1 -1 fxe fye cxe cye se];
disp('Errors estimates from checkerboards');
disp(rotAndIntrinsicsCheckerboardsErrors);

%% estimate rotation and intrinsics
% rotAndIntrinsics = [omega1 omega2 omega3 fx fy cx cy s]

%{ 
%errRK currently not parameterized correctly for this
% use fminsearch parameterized by (fx,fy,s,cx,cy,r1,r2,r3)
disp('SparkleCalibrate - fminsearch over the camera params');
rotAndIntrinsics1 = estimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos);
disp(rotAndIntrinsics1);
%} 

%%
% solve the linear system and do RQ decomposition to get K and R
disp('SparkleCalibrate - linear solution');
rotAndIntrinsics2 = linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos);
disp(rotAndIntrinsics2);
disp('difference with checkerboards');
disp(rotAndIntrinsicsCheckerboards - rotAndIntrinsics2);

%%
% solve linear system but with positions in coordinate system with origin
% at the center of the image
disp('SparkleCalibrate - linear solution with changed origin');
rotAndIntrinsics3 = diffOriginEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos);
disp(rotAndIntrinsics3);
disp('difference with checkerboards');
disp(rotAndIntrinsicsCheckerboards - rotAndIntrinsics3);

%{

disp('SparkleCalibrate - fminsearch parameterized by four corners');
rotAndIntrinsics4 = diffOriginEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos);
disp(rotAndIntrinsics4);
disp('difference with checkerboards');
disp(rotAndIntrinsicsCheckerboards - rotAndIntrinsics4);
%}

%%
%{ %
disp('SparkleCalibrate - linear solution as first guess in fminsearch (skew=0)');
rotAndIntrinsics5 = startPointEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos);
disp(rotAndIntrinsics5);
disp('difference with checkerboards');
disp(rotAndIntrinsicsCheckerboards - rotAndIntrinsics5);
%} %


%% save outputs
%TODO

