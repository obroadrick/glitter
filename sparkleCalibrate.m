% single image camera calibration with glitter

% % input: 
%  path to the single image
%  path to matfile with fiducial marker points
%  known point light source position
%  characterized sheet of glitter: spec locations, normals
% % output:
%  saves as matfile all the parameters estimated:
% intrinsic matrix, 
% rotation matrix, 
% rodrigues parameters,
% focal length x and y,
% skew,
% image center x and y,
% distortion paramters k1 and k2

wideangle1 = struct('name','Wide Angle Lens (light position off the monitor, chem side)', ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/wideAngleCardboard/', ...
            'impath','chem.JPG', ...
            'lightPosFname', 'chemLightPos.mat', ...
            'skew', true);
middle = struct('name','Original Camera (Nikonz7 35mm) middle position (light off monitor, chem side)', ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/', ...
            'impath','chem.JPG', ...
            'lightPosFname', 'chemLightPos.mat', ...
            'skew', false);
iphone1 = struct('name','iPhone XR (light off monitor, chem side)', ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/iphone/', ...
            'impath','chem.JPG', ...
            'lightPosFname', 'chemLightPos.mat', ...
            'skew', true);

for input=[wideangle1, middle, iphone1]
expdir = input.expdir;
impath = [expdir input.impath];
lightPos = matfile([expdir input.lightPosFname]).lightPos;
fprintf('%s\n',input.name);
skew = input.skew;

% path to single image fiducial marker points
if ~isfile([expdir '16pts.mat'])
    % if the 16pts for this experiment haven't already been found, then
    % find them now using Addy's script
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', impath, [expdir '16pts.mat']);
    system(cmd);
    disp('here 1!');
end
% now that the 16pts have been found, get them in a usable data structure
allPts = matfile([expdir '16pts.mat']).arr;
pin = allPts(1,:);
pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
pin = double([pinx' piny']);
fiducialMarkerPoints = pin;

%  characterized sheet of glitter: spec locations, normals
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

%% estimate translation
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin, expdir, -1, skew);

%% get checkerboard outputs for comparison
if skew
    camParams = matfile([expdir 'camParamsSkew']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrorsSkew']).camParamsErrors;
    camPos = matfile([expdir 'camPosSkew']).camPos;
    camRot = matfile([expdir 'camRotSkew']).camRot;
else
    camParams = matfile([expdir 'camParams']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrors']).camParamsErrors;
    camPos = matfile([expdir 'camPos']).camPos;
    camRot = matfile([expdir 'camRot']).camRot;
end
omega = undoRodrigues(camRot);
fx = camParams.Intrinsics.FocalLength(1);
fy = camParams.Intrinsics.FocalLength(2);
cx = camParams.Intrinsics.PrincipalPoint(1);
cy = camParams.Intrinsics.PrincipalPoint(2);
s = camParams.Intrinsics.Skew;
rotAndIntrinsicsCheckerboards = [camPos omega(1) omega(2) omega(3) fx fy cx cy s];
columnNames = ["Tx","Ty","Tz","omega1","omega2","omega3","fx","fy","cx","cy","s"];
fprintf(['%15s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n'], '', columnNames);
printRow('Checker est.', rotAndIntrinsicsCheckerboards);
% also get error estimates
fxe = camParamsErrors.IntrinsicsErrors.FocalLengthError(1);
fye = camParamsErrors.IntrinsicsErrors.FocalLengthError(2);
cxe = camParamsErrors.IntrinsicsErrors.PrincipalPointError(1);
cye = camParamsErrors.IntrinsicsErrors.PrincipalPointError(2);
se = camParamsErrors.IntrinsicsErrors.SkewError;
rotAndIntrinsicsCheckerboardsErrors = [-1 -1 -1 -1 -1 -1 fxe fye cxe cye se];
printRow('Checker err.', rotAndIntrinsicsCheckerboardsErrors);

%% solve the linear system and do RQ decomposition to get K and R
rotAndIntrinsics2 = [camPosEst linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos, expdir)];
% print outputs
printBreak();
printRow('Sparkle est.', rotAndIntrinsics2);
diff = rotAndIntrinsicsCheckerboards - rotAndIntrinsics2;
%printRow('Diff.', diff);
percentErrors = (rotAndIntrinsicsCheckerboards - rotAndIntrinsics2) ./ rotAndIntrinsicsCheckerboards .* 100;
printRow('Percent diff.', percentErrors);
posDiff = sqrt(sum(((camPosEst-camPos).^2)));
R2 = rod2mat(rotAndIntrinsics2(4),rotAndIntrinsics2(5),rotAndIntrinsics2(6));
Rerr = rotDiff(R2, camRot);
fprintf('               Position diff. (mm): %.2f     Rotation diff. (deg): %.3f\n', posDiff, Rerr);

end

%{ 
disp('SparkleCalibrate - linear solution as first guess in fminsearch (skew=0)');
rotAndIntrinsics5 = startPointEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos);
disp(rotAndIntrinsics5);
disp('difference with checkerboards');
disp(rotAndIntrinsicsCheckerboards - rotAndIntrinsics5);
disp('percent error (%)');
disp((rotAndIntrinsicsCheckerboards - rotAndIntrinsics5) ./ rotAndIntrinsicsCheckerboards .* 100);
R5 = rod2mat(rotAndIntrinsics5(1),rotAndIntrinsics5(2),rotAndIntrinsics5(3));
Rerr = rotDiff(R5, camRot);
disp('difference in rotation (degrees):');
disp(Rerr);
%} 

function printRow(rowName, rowVals)
    w = '10'; dec = '4';
    s = '%15s ';
    for i=1:size(rowVals,2)
        s = [s '%' w '.' dec 'f '];
    end
    s = [s '\n'];
    fprintf(s, rowName, rowVals);
end

function printBreak()
    fprintf('               -------------------------------------------------------------------------------------------------------------------------\n');
end
