% multiple light sources sparkle calibrate

%%
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


%%
% % set inputs

expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/oct25_nikonz7_35mm/';
%expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_far/';
%expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/newCamPosNov6_middle/';
charDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep18characterization(new-1)/';
%charDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep19characterization(new-2)/';
setPaths(charDir);

%pointLightImageIndex = 1;
for pointLightImageIndex=1:1
    % path to the single image
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july19characterization/circleOnMonitor/2022-07-19T13,54,52circle-calib-W1127-H574-S48.jpg';
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/glitter/DSC_3113.JPG';
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/singleImageAug18.JPG';
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/singleImageAug18.JPG';
    %impath = [expdir '1single.JPG'];
    %pointLightImageIndex = 1;
    impath = [expdir int2str(pointLightImageIndex) 'single.JPG'];
    ambientImage = rgb2gray(imread([expdir 'blank1.JPG']));
    
    % path to single image fiducial marker points
    % get by running Addy's Python script on the single image:
    %allPts = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/july25testNikonz7/16ptsJuly25.mat']).arr;
    %allPts = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/16ptsAug18.mat').arr;
    allPts = matfile([expdir '16pts.mat']).arr;
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
    %lightPos = [0 125-73 535];%july25nikonz7 %TODO store in exp dir
    %lightPos = [0 129-72.9 527];%aug18nikonz7
    
    % get light positions for multiple light sources
    w = 3840;
    h = 2160;
    xoff = 500;
    yoff = 300;
    positions = [xoff yoff; w-xoff h-yoff; xoff h-yoff; w-xoff yoff];
    r = 9;
    for ix=1:size(positions,1)
        positions(ix,:) = positions(ix,:) + r/2;
    end
    
    %lightPos = screenPosToWorldPos(positions(1,:), M);
    M = matfile([expdir 'measurements.mat']).M;
    %M = matfile([charDir 'measurements.mat']).M;
    lightPos = screenPosToWorldPos(positions(pointLightImageIndex,:), M);
    % estimate translation and distortion
    % todo/future version
    %warning('off','MATLAB:singularMatrix');
    set(0,'DefaultFigureVisible','on');
    skew=true;

    %% estimate translation
    [camPosEst, mostInliersSpecPos, mostInliersImageSpecPos] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew);

    %%
    %{ 
    % per-characteriztion checkerboard calibration info
    %% first retrieve the checkerboard calibration information for comparison along the way
    format shortG;%display numbers in more reasonable way
    camParams = matfile(P.camParams).camParams;
    camParamsErrors = matfile(P.camParamsErrors).camParamsErrors;
    camPos = matfile(P.camPos).camPos;
    camRot = matfile(P.camRot).camRot;
    %%%%%%%%%%%
    %}
    %% manually set path literals to control the checkerboard outputs being used
    %{
    format shortG;%display numbers in more reasonable way
    %expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/';
    camParams = matfile([expdir 'camParams']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrors']).camParamsErrors;
    camPos = matfile([expdir 'camPos']).camPos;
    camRot = matfile([expdir 'camRot']).camRot;
    %}
    %% manually set path literals to control the checkerboard outputs being used
    format shortG;%display numbers in more reasonable way
    %expir = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/';
    camParams = matfile([expdir 'camParamsSkew']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrorsSkew']).camParamsErrors;
    camPos = matfile([expdir 'camPosSkew']).camPos;
    camRot = matfile([expdir 'camRotSkew']).camRot;
    
    %%
    %%%%%%%%%%%
    % rotAndIntrinsics = [omega1 omega2 omega3 fx fy cx cy s]
    omega = undoRodrigues(camRot);
    fx = camParams.Intrinsics.FocalLength(1);
    fy = camParams.Intrinsics.FocalLength(2);
    cx = camParams.Intrinsics.PrincipalPoint(1);
    cy = camParams.Intrinsics.PrincipalPoint(2);
    s = camParams.Intrinsics.Skew;
    rotAndIntrinsicsCheckerboards = [camPos omega(1) omega(2) omega(3) fx fy cx cy s];
    columnNames = ["Tx","Ty","Tz","omega1","omega2","omega3","fx","fy","cx","cy","s"];
    fprintf(['%15' ...
        '' ...
        's %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n'], '', columnNames);
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
    return
    
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


end

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
