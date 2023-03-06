% Single-image camera calibration with glitter

% Automatically perform 10 calibrations for 10 camera positions
testcases = loadTestCases();
useOptimizedNormals = true;
if useOptimizedNormals
    resultsSaveName = 'optimizedSparkleResults';
else
    resultsSaveName = 'sparkleResults';
end
for index = 1:10
input = testcases.feb10(index);
expdir = input.expdir;
impath = [expdir input.impath];
lightPos = matfile([expdir input.lightPosFname]).lightPos;
skew = input.skew; % whether we assume zero skew
compare = false; % whether this script will output comparisons with checkerboard results
fprintf('\n%s\n',input.name);

% Find ArUco markers (if they haven't been detected and saved already)
if ~isfile([expdir '16pts.mat'])
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', impath, [expdir '16pts.mat']);
    system(cmd);
end
pin = loadPoints([expdir '16pts.mat'], input.sixteen);

% Visualize the detected ArUco marker points on a bright image
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

% Get characterization paths for spec locations and surface normals
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

%% Estimate translation
other.inlierThreshold = 15;
ambientImage = -1;
if useOptimizedNormals
    other.customNormals = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep18characterization(new-1)/optimized_normals.mat';
end
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other);

%% Estimate rotation and intrinsics
rotAndIntrinsics2 = [camPosEst linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos, expdir, skew)];

%% Save results
sparkleResults(index,:) = rotAndIntrinsics2';
save([expdir resultsSaveName num2str(i)], "rotAndIntrinsics2");

%% Optimize for the skew=0 case
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
%% Compare results with checkerboard-based estimates
if compare
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
printRow('Sparkle est.', rotAndIntrinsics2);
diff = rotAndIntrinsicsCheckerboards - rotAndIntrinsics2;
percentErrors = (rotAndIntrinsicsCheckerboards - rotAndIntrinsics2) ./ rotAndIntrinsicsCheckerboards .* 100;
printRow('Percent diff.', percentErrors);
posDiff = sqrt(sum(((camPosEst-camPos).^2)));
R2 = rod2mat(rotAndIntrinsics2(4),rotAndIntrinsics2(5),rotAndIntrinsics2(6));
Rerr = rotDiff(R2, camRot);
fprintf('               Position diff. (mm): %.2f     Rotation diff. (deg): %.3f\n', posDiff, Rerr);
end % end if compare
end % end for loop
%%
% Save all the sparkle results in one file
for i=1:10
    sparkleResults(i,:) = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/' resultsSaveName num2str(i)]).rotAndIntrinsics2;
end
save(['/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/' resultsSaveName], "sparkleResults");

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