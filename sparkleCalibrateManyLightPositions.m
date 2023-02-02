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
warning('off','MATLAB:singularMatrix');

wideangle1 = struct('name','Wide Angle Lens (light position off the monitor, chem side)', ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/wideAngleCardboard/', ...
            'impath','chem.JPG', ...
            'lightPosFname', 'chemLightPos.mat', ...
            'skew', true, ...
            'sixteen', false);
middle = struct('name','Original Camera (Nikonz7 35mm) middle position (light off monitor, chem side)', ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/', ...
            'impath','chem.JPG', ...
            'lightPosFname', 'chemLightPos.mat', ...
            'skew', false, ...
            'sixteen', true);
iphone1 = struct('name','iPhone XR (light off monitor, chem side)', ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/iphone/', ...
            'impath','chem.JPG', ...
            'lightPosFname', 'chemLightPos.mat', ...
            'skew', true, ...
            'sixteen', true);

for index = 1:1
%{
jan12_1 = struct('name',['Nikon Z7 (January 12 Data) Position ' num2str(index)], ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/', ...
            'impath',[num2str(index) '.JPG'], ...
            'lightPosFname', ['lightPos' num2str(num2str(index)) '.mat'], ...
            'skew', true, ...
            'sixteen', true);
%}
jan12 = struct('name',['Nikon Z7 (January 12 Data) Position ' num2str(index)], ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/', ...
            'impath',[num2str(index) '.JPG'], ...
            'lightPosFname', ['lightPos' num2str(num2str(index)) '.mat'], ...
            'skew', true, ...
            'sixteen', true);
jan13 = struct('name',['Nikon Z7 (January 13 Data) Position ' num2str(index)], ...
            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/', ...
            'impath',[num2str(index) '.JPG'], ...
            'lightPosFname', ['lightPos' num2str(num2str(index)) '.mat'], ...
            'skew', true, ...
            'sixteen', true);


input = jan13;
%for input=[wideangle1, middle, iphone1]
expdir = input.expdir;
impath = [expdir input.impath];
lightPos = matfile([expdir input.lightPosFname]).lightPos;
fprintf('\n%s\n',input.name);
skew = input.skew;

% path to single image fiducial marker points
if ~isfile([expdir '16pts.mat'])
    % if the 16pts for this experiment haven't already been found, then
    % find them now using Addy's script
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', impath, [expdir '16pts.mat']);
    system(cmd);
end
% now that the 16pts have been found, get them in a usable data structure
allPts = matfile([expdir '16pts.mat']).arr;
pin = [allPts(1,:) allPts(2,:) allPts(3,:) allPts(4,:)];
for i=1:16
    pinx(i) = pin{i}(1);
    piny(i) = pin{i}(2);
end
pin = double([pinx' piny']);
fiducialMarkerPoints = pin;
if ~(input.sixteen)
    pin = pin(1:4,:);
end

if input.sixteen && isfile([expdir 'bright.JPG'])
    % visualize the fiducial marker points (both pin and pout)
    figure; tiledlayout(1,2,"TileSpacing","tight","Padding","tight");
    nexttile;title('Fiducial points in image coordinates');
    imagesc(imread([expdir 'bright.JPG'])); hold on;
    for i=1:16
        plot(pin(i,1), pin(i,2), 'rx');
        text(pin(i,1), pin(i,2), int2str(i));
    end
    nexttile; hold on; title('Fiducial points in canonical glitter coordinates');
    pout = getFiducialMarkerPts();
    for i=1:16
        plot(pout(i,1), pout(i,2), 'rx');
        text(pout(i,1), pout(i,2), int2str(i));
    end
end

%  characterized sheet of glitter: spec locations, normals
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

other.inlierThreshold = 15;
ambientImage = -1;
%% estimate translation
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other);

%{
%i = 1;
%inlierThresholds = [5, 10, 15, 20, 25, 30, 35, 40, 45];
%for inlierThreshold = inlierThresholds
    other.inlierThreshold = inlierThreshold;
    [camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin, expdir, -1, skew, other);
    %camPosEstList(i,:) = camPosEst;
    %numInliersList(i,:) = size(mostInliersSpecPos, 1);
    %i = i + 1;
%end
%errs2 = sqrt(sum((camPosEstList - camPos).^2, 2));
%inliers2 = numInliersList;
%}

%% get checkerboard outputs for comparison
if skew
    camParams = matfile([expdir '/' num2str(index) '/camParamsSkew']).camParams;
    camParamsErrors = matfile([expdir '/' num2str(index) '/camParamsErrorsSkew']).camParamsErrors;
    camPos = matfile([expdir '/' num2str(index) '/camPosSkew']).camPos;
    camRot = matfile([expdir '/' num2str(index) '/camRotSkew']).camRot; 
    camPosErr = matfile([expdir '/' num2str(index) '/camPosErrSkew']).camPosErr;
    camRotErr = matfile([expdir '/' num2str(index) '/camRotErrSkew']).camRotErr;
else
    camParams = matfile([expdir 'camParams']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrors']).camParamsErrors;
    camPos = matfile([expdir 'camPos']).camPos;
    camRot = matfile([expdir 'camRot']).camRot;
    camPosErr = matfile([expdir '/' num2str(index) '/camPosErr']).camPosErr;
    camRotErr = matfile([expdir '/' num2str(index) '/camRotErr']).camRotErr;
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
rotAndIntrinsicsCheckerboardsErrors = [camPosErr camRotErr fxe fye cxe cye se];
printRow('Checker err.', rotAndIntrinsicsCheckerboardsErrors);

%% solve the linear system and do RQ decomposition to get K and R
rotAndIntrinsics2 = [camPosEst linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos, expdir, skew)];
% print outputs
%printBreak();
printRow('Sparkle est.', rotAndIntrinsics2);
diff = rotAndIntrinsicsCheckerboards - rotAndIntrinsics2;
%printRow('Diff.', diff);
percentErrors = (rotAndIntrinsicsCheckerboards - rotAndIntrinsics2) ./ rotAndIntrinsicsCheckerboards .* 100;
printRow('Percent diff.', percentErrors);
posDiff = sqrt(sum(((camPosEst-camPos).^2)));
R2 = rod2mat(rotAndIntrinsics2(4),rotAndIntrinsics2(5),rotAndIntrinsics2(6));
Rerr = rotDiff(R2, camRot);
fprintf('               Position diff. (mm): %.2f     Rotation diff. (deg): %.3f\n', posDiff, Rerr);

sparkleResults(index,:) = rotAndIntrinsics2';
save([expdir 'sparkleResults' num2str(index)], "rotAndIntrinsics2");

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