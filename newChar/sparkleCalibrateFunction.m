% single image camera calibration with glitter as a function
function results = sparkleCalibrateFunction(input, compare)
% input example:
%wideangle1 = struct('name','Wide Angle Lens (light position off the monitor, chem side)', ...
%            'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/wideAngleCardboard/', ...
%            'impath','chem.JPG', ...
%            'lightPosFname', 'chemLightPos.mat', ...
%            'skew', true, ...
%            'sixteen', false);

warning('off','MATLAB:singularMatrix');

% get the individual items within the input
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
pin = getPoints([expdir '16pts.mat']);
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

%% estimate translation
other.inlierThreshold = 15;
ambientImage = -1;
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other);

if compare
    % get checkerboard outputs for comparison
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
end

%% solve the linear system and do RQ decomposition to get K and R
rotAndIntrinsics2 = [camPosEst linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos, expdir, skew)];
% print outputs
%printBreak();
printRow('Sparkle est.', rotAndIntrinsics2);
R2 = rod2mat(rotAndIntrinsics2(4),rotAndIntrinsics2(5),rotAndIntrinsics2(6));
if compare
    percentErrors = (rotAndIntrinsicsCheckerboards - rotAndIntrinsics2) ./ rotAndIntrinsicsCheckerboards .* 100;
    printRow('Percent diff.', percentErrors);
    posDiff = sqrt(sum(((camPosEst-camPos).^2)));
    Rerr = rotDiff(R2, camRot);
    fprintf('               Position diff. (mm): %.2f     Rotation diff. (deg): %.3f\n', posDiff, Rerr);
end
results = rotAndIntrinsics2';

end %end sparkleCalibrateFunction

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