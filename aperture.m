% Single-image camera calibration with glitter ... exploring aperture


input = struct('name',['Nikon Z7 (February 16 Data)'], ...
                'expdir','/Users/oliverbroadrick/Desktop/glitter-stuff/feb16/', ...
                'lightPosFname', ['lightPos1.mat'], ...
                'sixteen', true, ...
                'numLightPositions', 1);
% Get file names
dirPath = [input.expdir];
allFiles = dir(dirPath);
allFiles = allFiles(~ismember({allFiles.name},{'.','..'}));
fIdx = 1;
for j=1:size(allFiles,1)
    % skip over .mat files
    if ~isempty(regexp(allFiles(j).name,'.mat')) || ~isempty(regexp(allFiles(j).name,'DS_Store'))
        continue
    end
    imageFileNames{fIdx} = [allFiles(j).folder '/' allFiles(j).name];
    imageFileJustNames{fIdx} = [allFiles(j).name];
    fIdx = fIdx + 1;
end

%{
% Show images we're working with here
s = ceil(sqrt(size(imageFileNames,2)));
figure; t= tiledlayout(s,s,'TileSpacing','tight','Padding','tight');
title(t,dirPath);
for ix=1:size(imageFileNames,2)
    disp(imageFileNames{ix})
    nexttile;
    hold on;title(imageFileJustNames{ix}(1:size(imageFileJustNames{ix},2)-4),'interpreter','none');
    resizeFactor = .01;
    imagesc(imresize(imread(imageFileNames{ix}),resizeFactor));
    %plot(imagePoints(:,1,ix).*resizeFactor,imagePoints(:,2,ix).*resizeFactor,'CX')%,'linewidth',2,'markersize',10);
    %set(gca,'xtick',[],'ytick',[]);%,'xticklabel',[],'yticklabel',[]);
    set(gca,'visible','off','xtick',[],'ytick',[],'YDir','reverse');
    set(findall(gca, 'type', 'text'), 'visible', 'on');
    drawnow;
end
%}

%%

for index=1:size(imageFileNames,2)

expdir = input.expdir;
impath = imageFileNames{index};
lightPos = matfile([expdir input.lightPosFname]).lightPos;
%skew = input.skew; % whether we assume zero skew
skew=false;
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
other.compare = false;
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos, other] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other);
reflectedRayToPinholeDists{index} = other.reflectedRayToPinholeDists;
mostInliersIntensitys{index} = other.mostInliersIntensitys;

continue

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

% visualize the histograms of reflectedRayToPinholeDists for each index
figure;
w=ceil(sqrt(size(reflectedRayToPinholeDists,2)));
tiledlayout(w,w,'TileSpacing','tight','Padding','tight');
for index=1:size(reflectedRayToPinholeDists,2)
    nexttile;
    histogram(reflectedRayToPinholeDists{index},10);
    xlabel('mm to pinhole');
end


%%
%{
% Save all the sparkle results in one file
for i=1:10
    sparkleResults(i,:) = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/' resultsSaveName num2str(i)]).rotAndIntrinsics2;
end
save(['/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/' resultsSaveName], "sparkleResults");
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