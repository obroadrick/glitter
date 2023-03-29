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
resultsSaveName = 'sparkleResults';
testcases = loadTestCases();
for index = 1:6
%input = testcases.mar4tests(index);
%input = testcases.ICCV_camPos1(index);
input = testcases.c2(index);
%for input=[wideangle1, middle, iphone1]
expdir = input.expdir;
imname = [input.impath];
%imname = '1b.JPG';
impath = [expdir imname];
lightPos = matfile([expdir input.lightPosFname]).lightPos;
fprintf('\n%s. ',input.name);
skew = input.skew;

other.perImageNaming = false;
[transformPath, pin, worldPoints] = getCheckerboardHomography(expdir, imname, other);
%%
plotStuff = false;%TODO adapt this for this test case
if plotStuff
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
P = getMar4charPaths();

other.inlierThreshold = 20;
ambientImage = -1;
other.compare = false;
other.quickEstimate = true;
%% estimate translation
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos, other] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other);

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

%% visualize stuff
visualize = true;
if visualize
    figure;
    hold on;
    % plot checkerboard points
    plot3(worldPoints(:,1), worldPoints(:,2), zeros(size(worldPoints,1),1),'b+');
    % plot estCamPos
    plot3(camPosEst(1), camPosEst(2), camPosEst(3), 'b*', MarkerSize=10, LineWidth=2);
    % plot 'known' camPos
    plot3(camPos(1), camPos(2), camPos(3), 'gs', MarkerSize=10, LineWidth=2);
    % plot light pos
    plot3(lightPos(1), lightPos(2), lightPos(3), 'rx', MarkerSize=10, LineWidth=2);
end

%% visualize more stuff
visualize = false;
if visualize
    figure;
    tiledlayout(1,2,'TileSpacing','tight','Padding','tight');
    ax1=nexttile;
    hold on;
    worldSpecPos = other.specPos;
    [transformPath, imagePoints, ~] = getCheckerboardHomography(expdir, impath);
    tform = matfile(transformPath).tform;
    imSpecPos = transformPointsInverse(tform,worldSpecPos(:,1:2));
    imSpecPosInliers = transformPointsInverse(tform,mostInliersSpecPos(:,1:2));
    plot(imagePoints(:,1), imagePoints(:,2), 'b+','MarkerSize',10,'LineWidth',1.5);
    set(gca,'YDir','reverse');
    % plot checkerboard points
    plot(imSpecPos(:,1), imSpecPos(:,2), 'r*','MarkerSize',10,'LineWidth',1.5)
    % plot all sparkles
    %plot(other.specPos(:,1), other.specPos(:,2), 'r*')
    % plot inlier sparkles
    %plot(mostInliersSpecPos(:,1), mostInliersSpecPos(:,2), 'b*')
    plot(imSpecPosInliers(:,1), imSpecPosInliers(:,2), 'b*','MarkerSize',10,'LineWidth',1.5)
    ax2=nexttile;imagesc(imread(impath));
    linkaxes([ax1 ax2]);
end

%% solve the linear system and do RQ decomposition to get K and R
other.plotStuff = true;
worldFiducials = worldPoints; 
imageFiducials = pin;
rotAndIntrinsics2 = [camPosEst linearEstimateRKglitter(impath, camPosEst, imageFiducials, worldFiducials, mostInliersSpecPos, mostInliersImageSpecPos, expdir, skew, other)];
% save results
sparkleResults(index,:) = rotAndIntrinsics2';
%save([expdir resultsSaveName num2str(i)], "rotAndIntrinsics2");
% print outputs
%printBreak();
printRow('Sparkle est.', rotAndIntrinsics2);
diff = rotAndIntrinsicsCheckerboards - rotAndIntrinsics2;
printRow('Diff.', diff);
percentErrors = (rotAndIntrinsicsCheckerboards - rotAndIntrinsics2) ./ rotAndIntrinsicsCheckerboards .* 100;
printRow('Percent diff.', percentErrors);
printRow('Expected.', [-1 -1 -1 -1 -1 -1 -1 -1 size(imread(impath),2)/2 size(imread(impath),1)/2 0]);
posDiff = sqrt(sum(((camPosEst-camPos).^2)));
R2 = rod2mat(rotAndIntrinsics2(4),rotAndIntrinsics2(5),rotAndIntrinsics2(6));
Rerr = rotDiff(R2, camRot);
fprintf('               Position diff. (mm): %.2f     Rotation diff. (deg): %.3f\n', posDiff, Rerr);
%end
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
end% end loop

save([expdir resultsSaveName], "sparkleResults");

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