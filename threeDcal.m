% 3d calibration - don't solve that tricky many-planar-patterns problem,
% just use known relative positions of points!

expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3d-cal-ims/';
d = dir([expdir '*.JPG']);
d = d(~ismember({d.name},{'.','..'}));
numIms = size(d,1);

% Load in images
for i=1:numIms
    imnames{i} = [d(i).name];
    impaths{i} = [d(i).folder '/' d(i).name];
    ims{i} = imread(impaths{i});
end

%%
% Find the ArUco points in each image
for i=1:numIms
    %if ~isfile([expdir '/' imnames{i} '_16pts.mat'])
        % Use Addy's Python scipt to detect the ArUco markers
        setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
        cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/detectArUcosNew.py "%s" "%s"', impaths{i}, [expdir '/' imnames{i} '_16pts.mat']);
        system(cmd);
    %eend
end

%%
% Display images with detected ArUco marker points
%{
figure;
tiledlayout(floor(sqrt(numIms)), ceil(sqrt(numIms)), 'TileSpacing','tight','Padding','tight');
for i=1:numIms
    % Show image
    ax = nexttile;
    imagesc(ims{i}); hold on;

    % Load and plot ArUco points
    if ~isfile([expdir '/' imnames{i} '_16pts.mat'])
        continue
    end
    pts{i} = loadPoints([expdir '/' imnames{i} '_16pts.mat'],true);
    for ix=1:size(pts{i},1)
        if pts{i}(ix,1) > 0 && pts{i}(ix,2) > 0
            plot(pts{i}(ix,1), pts{i}(ix,2),'cX','MarkerSize', 20);
            text(pts{i}(ix,1), pts{i}(ix,2),num2str(ix));
        end
    end

    % Draw plot
    set(ax,'xticklabel',[],'yticklabel',[])
    title(d(i).name);
    drawnow;
end
%}

%%
% Get the corresponding points in the world/glitter coordinate system
for i=1:numIms
    if ~isfile([expdir '/' imnames{i} '_16pts.mat'])
        arucoPoints{i} = -1;
        continue
    end
    
    % Load in the detected ArUco points in the image
    arucoPoints{i} = loadPoints([expdir '/' imnames{i} '_16pts.mat'],true);

    % Get the position of the markers on the glitter sheet
    p = getFiducialMarkerPts(false); 
    p = [p(:,1) p(:,2) zeros(size(p,1),1)];

    % Adjust for the position of the glitter sheet on the table
    % (the file name encodes the position on the table)
    direction = 1;
    if imnames{i}(1) == 'n'
        direction = -1;
    end
    distance = str2double(imnames{i}(2:3));
    MM_PER_INCH = 25.4;
    adjustment_inches = distance * direction * MM_PER_INCH;

    worldPoints{i} = p + [0 0 adjustment_inches];
end

%% 
% Get all the legitimate correspondences (remove those which are
% negative ones from failed aruco point detection). Also, if any two of the
% four correspondences for a given marker are the same, then ignore that
% set of four correspondences.
numPts = 1;
for i=1:numIms
    if arucoPoints{i} == -1
        keptImPoints{i} = zeros(0,0);
        keptWorldPoints{i} = zeros(0,0);
        continue
    end
    % If the points are not all unique, ignore this image (failure of our
    % detection code)
    %{
    if ~(size(unique(arucoPoints{i},'rows'), 1) == size(arucoPoints{i}, 1))
        keptImPoints{i} = zeros(0,0);
        keptWorldPoints{i} = zeros(0,0);
        disp('here')
        continue
    end
    %}
    thisImNumPts = 1;
    for j=1:size(arucoPoints{i},1)
        if ~(arucoPoints{i}(j,1) > 0 && arucoPoints{i}(j,2) > 0)
            continue
        end
        imPts(numPts,:) = arucoPoints{i}(j,:);
        worldPts(numPts,:) = worldPoints{i}(j,:);
        numPts = numPts + 1;
        keptImPoints{i}(thisImNumPts,:) = arucoPoints{i}(j,:);
        keptWorldPoints{i}(thisImNumPts,:) = worldPoints{i}(j,:);
        thisImNumPts = thisImNumPts + 1;
    end
end

%%
% Get the camera matrix
%[C, r] = estimateCameraMatrix(imPts,worldPts);
C = solveForCameraMatrix(imPts,worldPts);

%%
% Decompose
[K,R,IminusC]= Pdecomp(C);
T = -IminusC(:,4);
camPos3d = T;
camRot3d = R;
K = K ./ K(3,3);
omega = mat2rod(R);
fx = K(1,1); fy = K(2,2); cx = K(1,3); cy = K(2,3); s = K(1,2);
intrinsics =  [omega fx fy cx cy s];
results = [camPos3d' intrinsics];

%{ 
%%
% Using the linear system solution as an initial guess, do a simplix search
% with the reprojection error as the loss function. 
% (Turns out doing so doesn't change the reprojection much... (i.e., the 
% reprojection error is already near-minimum when you solve the linear 
% system.).)
% Optimize over the 11 entries in the camera matrix
f = @(x) reprojectionErrorC(x, imPts, worldPts);
options = optimset('PlotFcns',@optimplotfval);
tuned_results = fminsearch(f, C(1:11), options);
%}

%% 
% Also estimate the radial distortion by using the solution to the linear
% system to initialize the rest of the parameters and doing simplex search
% over all of the parameters at once using the reprojection error as the
% loss function.
%{
f = @(x) reprojectionErrorRadialDistortion(x(1:11), x(12:13), imPts, worldPts);
options = optimset('PlotFcns',@optimplotfval);
tuned_results = fminsearch(f, [C(1:11) 0 0], options);
%}

%%
% Save results
save('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults.mat',"results");

%%
% Compute reprojection errors in pixels
for i=1:size(imPts,1)
    x = imPts(i,:);
    X = [worldPts(i,:)'; 1]; % in homogeneous coordinates
    y = C * X;
    y = y ./ y(3);
    reprojectedPos(i,:) = y(1:2)';
    err(i) = norm(x'-y(1:2));
end

%%
% Display images with detected ArUco marker points and with reprojected
% points according to this camera matrix
figure;
tiledlayout(1,3, 'TileSpacing','tight','Padding','tight');
for i=[2,5,13]
    % Show image
    ax = nexttile;
    imagesc(ims{i}); hold on;

    for ix=1:size(keptImPoints{i},1)
        % Show the detected ArUco image point
        plot(keptImPoints{i}(ix,1), keptImPoints{i}(ix,2),'cX','MarkerSize', 20, 'LineWidth',2);

        % Show the world point reprojected according to our camera matrix
        reproj = reproject(keptWorldPoints{i}(ix,:)',C);
        plot(reproj(1), reproj(2), 'r+', 'MarkerSize', 15, 'LineWidth',2);
    end

    % Draw plot
    set(ax,'xticklabel',[],'yticklabel',[]);
    title(d(i).name);
    drawnow;
end

%%
% What if we take random subsets of the available points? This way we get
% at least some sense of the distribution of estimates by this calibration
% method...
% Divide into N random subsets (of roughly equal size)
return
N = 4;
n = size(imPts,1); 
m = floor(n/N);
idx = randperm(n);
for ix=1:N
    % Get this random subset of the point correspondences
    imPtsCur = imPts(idx((ix-1)*floor(n/N)+1:ix*floor(n/N)),:);
    worldPtsCur = worldPts(idx((ix-1)*floor(n/N)+1:ix*floor(n/N)),:);
    % Estimate the camera matrix based on these correspondences
    [C, r] = estimateCameraMatrix(imPtsCur,worldPtsCur);
    C = C';
    % Decompose
    [K,R,IminusC]= Pdecomp(C);
    camPos3d = -IminusC(:,4);
    camRot3d = R;
    K = K ./ K(3,3);
    omega = mat2rod(camRot3d);
    fx = K(1,1); fy = K(2,2); cx = K(1,3); cy = K(2,3); s = K(1,2);
    intrinsics =  [omega fx fy cx cy s];
    results(ix,:) = [camPos3d' intrinsics];
end
save(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults' num2str(N) '.mat'],"results");

%%

%{
%% See how checkerboard parameters and glitter parameters reproject these 
% 3d points correspondences onto the image...
sparkleResults = matfile([expdir 'sparkleResults.mat']).results;
checkerResults = matfile([expdir 'checkerResults.mat']).results;
for i=1:10
    sparkleE(i) = reprojectionError(sparkleResults(i,:),imPts,worldPts);
    checkerE(i) = reprojectionError(checkerResults(i,:),imPts,worldPts);

    % Let matlab reproject the point (using distortion too)
    camParams = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/camParams.mat']).camParams;
    intrinsics = camParams.Intrinsics;
    rotationMatrix = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/camRot.mat']).camRot;
    rotationMatrix = rotationMatrix';
    camPos = matfile(['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(i) '/camPos.mat']).camPos;
    translationVector = -rotationMatrix * camPos';
    e = 0;
    for ix=1:size(imPts,1)
        reproj = worldToImage(intrinsics,rotationMatrix,translationVector,worldPts(ix,:));
        disp(reproj);disp(imPts(i,:));
        e = e + norm(reproj - imPts(i,:));
    end
    e = e  / size(imPts,1);
    checkerEdist(i) = e
end
%}

%% 
% Also estimate the radial distortion by using the solution to the linear
% system to initialize the rest of the parameters and doing simplex search
% over all of the parameters at once using the reprojection error as the
% loss function.
%{
f = @(x) reprojectionErrorRadialDistortion(x(1:11), x(12:13), imPts, worldPts);
options = optimset('PlotFcns',@optimplotfval);
tuned_results = fminsearch(f, [C(1:11) 0 0], options);
% What if you just optimized over the radial distortion coefficients
% (keeping the rest of the parameters fixed as found by linear solution)...
f = @(x) reprojectionErrorRadialDistortion(C(1:11), x, imPts, worldPts);
options = optimset('PlotFcns',@optimplotfval);
tuned_results = fminsearch(f, [0 0], options);
%}
%{
% What if we optimized over the very relative positions of the ArUco
% markers themselves?
f = @(x) reprojectionErrorMoveMarkers(x, imPts, imnames, arucoPoints);
options = optimset('PlotFcns',@optimplotfval);
x0 = reshape(getFiducialMarkerPts(), [], 1);
tuned_results = fminsearch(f, x0, options);
%}
% What if we optimized over the very 3d relative positions of the ArUco
% markers themselves?
%{
f = @(x) reprojectionErrorMoveMarkers3d(x, imPts, imnames, arucoPoints);
options = optimset('PlotFcns',@optimplotfval);
x0 = reshape([getFiducialMarkerPts() zeros(size(getFiducialMarkerPts(),1),1)], [], 1);
tuned_results = fminsearch(f, x0, options);
%}

%{
%%
% TEMPORARY IN-LINE EXECUTION OF THE ABOVE LOSS FUNCTION
f(tuned_results);
x=tuned_results;
fiducialPoints = reshape(x, [size(getFiducialMarkerPts(),1) 3]);
% Get new world points based on the new relative fiducial marker points
% given by x
%fiducialPoints = [fiducialPoints zeros(size(fiducialPoints,1),1)];
worldPts = getWorldPoints(fiducialPoints, imnames, arucoPoints);
disp(fiducialPoints);
% Solve linear system for camera matrix (ignore radial distortion so it
% is quick)
%C = solveForCameraMatrix(imPts, worldPts);
C = estimateCameraMatrix(imPts, worldPts)';

% Find average reprojection error
e = 0;
for i=1:size(imPts,1)
    X = worldPts(i,:)';
    x = reproject(X,C);
    e = e + norm(x-imPts(i,:)');
end

% Divide by number of points reprojected so that e gives the average
% reprojection error (more interpretable)
e = e / size(imPts,1);
%}


%{
xf = tuned_results;
x0 = reshape(x0, size(getFiducialMarkerPts()));
xf = reshape(xf, size(getFiducialMarkerPts()));
figure;
t = tiledlayout(2,2,"TileSpacing",'tight','Padding','tight');
title(t,'avg reproj error from 6.2 to 3.5');
for i=1:4
    nexttile
    plot(x0(:,1),x0(:,2),'r+','LineWidth',2);
    hold on;
    plot(xf(:,1),xf(:,2),'gx','LineWidth',2);
    legend('measured','optimized');
end
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% functions: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r = reproject(X,C)
    y = C * [X; 1];
    r = y ./ y(3);
    r = r(1:2);
end

function e = reprojectionError(results,imPts,worldPts)
    T = results(1:3)';
    R = rod2mat(results(4),results(5),results(6));
    K = [results(7) results(11) results(9); 0 results(8) results(10); 0 0 1];
    e = 0;
    for i=1:size(imPts,1)
        x = K * R * (worldPts(i,:)' - T);
        x = x ./ x(3);
        x = x(1:2);
        e = e + norm(x-imPts(i,:)');
    end
    % Divide by number of points reprojected so that e gives the average
    % reprojection error (more interpretable)
    e = e / size(imPts,1);
end

function e = reprojectionErrorC(C,imPts,worldPts)
    % Reconstruct the full camera matrix from the 11 entries passed
    C = [C(1:3)' C(4:6)' C(7:9)' [C(10:11) 1]'];
    % Reproject all the passed world points, summing up the difference
    % between the reprojections and the passed corresponding image points
    e = 0;
    for i=1:size(imPts,1)
        x = C * [worldPts(i,:)'; 1];
        x = x ./ x(3);
        x = x(1:2);
        e = e + norm(x-imPts(i,:)');
    end
    % Divide by number of points reprojected so that e gives the average
    % reprojection error (more interpretable)
    e = e / size(imPts,1);
end

function e = reprojectionErrorRadialDistortion(C, k, imPts, worldPts)
% Reconstruct the full camera matrix from the 11 entries passed
    C = [C(1:3)' C(4:6)' C(7:9)' [C(10:11) 1]'];
    % Reproject all the passed world points, summing up the difference
    % between the reprojections and the passed corresponding image points
    e = 0;
    [K,R,IminusC]= Pdecomp(C);
    T = -IminusC(:,4);
    for i=1:size(imPts,1)
        worldPt = worldPts(i,:)';
        x = reprojectRadialDistortion(K,R,T,k,worldPt);
        e = e + norm(x-imPts(i,:)');
    end
    % Divide by number of points reprojected so that e gives the average
    % reprojection error (more interpretable)
    e = e / size(imPts,1);
end

function e = reprojectionErrorMoveMarkers(x, imPts, imnames, arucoPoints)
    fiducialPoints = reshape(x, size(getFiducialMarkerPts()));
    % Make 3d
    fiducialPoints = [fiducialPoints zeros(size(fiducialPoints,1),1)];
    % Get new world points based on the new relative fiducial marker points
    % given by x
    worldPts = getWorldPoints(fiducialPoints, imnames, arucoPoints);

    % Solve linear system for camera matrix (ignore radial distortion so it
    % is quick)
    C = solveForCameraMatrix(imPts, worldPts);

    % Find average reprojection error
    e = 0;
    for i=1:size(imPts,1)
        X = worldPts(i,:)';
        x = reproject(X,C);
        e = e + norm(x-imPts(i,:)');
    end

    % Divide by number of points reprojected so that e gives the average
    % reprojection error (more interpretable)
    e = e / size(imPts,1);
end

function e = reprojectionErrorMoveMarkers3d(x, imPts, imnames, arucoPoints)
    fiducialPoints = reshape(x, [size(getFiducialMarkerPts(),1) 3]);
    % Get new world points based on the new relative fiducial marker points
    % given by x
    %fiducialPoints = [fiducialPoints zeros(size(fiducialPoints,1),1)];
    worldPts = getWorldPoints(fiducialPoints, imnames, arucoPoints);
    disp(fiducialPoints);
    % Solve linear system for camera matrix (ignore radial distortion so it
    % is quick)
    %C = solveForCameraMatrix(imPts, worldPts);
    C = estimateCameraMatrix(imPts, worldPts)';

    % Find average reprojection error
    e = 0;
    for i=1:size(imPts,1)
        X = worldPts(i,:)';
        x = reproject(X,C);
        e = e + norm(x-imPts(i,:)');
    end

    % Divide by number of points reprojected so that e gives the average
    % reprojection error (more interpretable)
    e = e / size(imPts,1);
end

function p = reprojectRadialDistortion(K,R,T,k,worldPt)
    % Compute the linear projection of worldPt onto the image
    x = K * R * (worldPt - T);
    x = x ./ x(3);
    x = x(1:2);

    % Apply radial distortion:
    % Convert into normalized image coordinates
    x = x - [K(1,3) K(2,3)]'; % translate to optical center
    x = x ./ [K(1,1) K(2,2)]';% divide by focal length
    % Get updated position based on radial distortion model
    if size (k < 3)
        k(3) = 0;
    end
    rsquared = x(1)^2 + x(2)^2;
    x = x .* (1 + k(1)*rsquared + k(2)*rsquared^2 + k(3)*rsquared^3);
    % Convert back into image coordinates
    x = x .* [K(1,1) K(2,2)]';
    x = x + [K(1,3) K(2,3)]';

    p = x(1:2);
end

function P = solveForCameraMatrix(imPts,worldPts)
    % Solve the linear system to get the camera parameters using these
    % correspondences
    for i=1:size(imPts,1)
        C(i*2-1,:) = [worldPts(i,1) worldPts(i,2) worldPts(i,3) 1 0 0 0 0 -worldPts(i,1)*imPts(i,1) -worldPts(i,2)*imPts(i,1) -worldPts(i,3)*imPts(i,1)];
        C(i*2,:) = [0 0 0 0 worldPts(i,1) worldPts(i,2) worldPts(i,3) 1 -worldPts(i,1)*imPts(i,2) -worldPts(i,2)*imPts(i,2) -worldPts(i,3)*imPts(i,2)];
        b(i*2-1,:) = [imPts(i,1)];
        b(i*2,:) = [imPts(i,2)];
        %{
        b(i*3-2,:) = [imPts(i,1)];
        b(i*3-1,:) = [imPts(i,2)];
        b(i*3,:) = [1];
        %}
    end
    x = C\b;
    P = [x(1:4)';x(5:8)';[x(9:11)' 1]];
end

function worldPts = getWorldPoints(fiducialMarkerPts, imnames, arucoPoints)
    %
    % Get the corresponding points in the world/glitter coordinate system
    for i=1:size(imnames,2)
        % Get the position of the markers on the glitter sheet
        p = fiducialMarkerPts;
        %p = [p(:,1) p(:,2) zeros(size(p,1),1)];
    
        % Adjust for the position of the glitter sheet on the table
        % (the file name encodes the position on the table)
        direction = 1;
        if imnames{i}(1) == 'n'
            direction = -1;
        end
        distance = str2double(imnames{i}(2:3));
        MM_PER_INCH = 25.4;
        adjustment_inches = distance * direction * MM_PER_INCH;
    
        worldPoints{i} = p + [0 0 adjustment_inches];
    end
    
    % Get all the legitimate correspondences (remove those which are
    % negative ones from failed aruco point detection). Also, if any two of the
    % four correspondences for a given marker are the same, then ignore that
    % set of four correspondences.
    numPts = 1;
    for i=1:size(imnames,2)
        if arucoPoints{i} == -1
            keptImPoints{i} = zeros(0,0);
            keptWorldPoints{i} = zeros(0,0);
            continue
        end
        % If the points are not all unique, ignore this image (failure of our
        % detection code)
        %{
        if ~(size(unique(arucoPoints{i},'rows'), 1) == size(arucoPoints{i}, 1))
            keptImPoints{i} = zeros(0,0);
            keptWorldPoints{i} = zeros(0,0);
            disp('here')
            continue
        end
        %}
        thisImNumPts = 1;
        for j=1:size(arucoPoints{i},1)
            if ~(arucoPoints{i}(j,1) > 0 && arucoPoints{i}(j,2) > 0)
                continue
            end
            imPts(numPts,:) = arucoPoints{i}(j,:);
            worldPts(numPts,:) = worldPoints{i}(j,:);
            numPts = numPts + 1;
            keptImPoints{i}(thisImNumPts,:) = arucoPoints{i}(j,:);
            keptWorldPoints{i}(thisImNumPts,:) = worldPoints{i}(j,:);
            thisImNumPts = thisImNumPts + 1;
        end
    end
end

% Compute the camera position and rotation (in the ways we usually use
% them) so that we can compare against the other calibration methods
%{
% At first of course I tried and failed to do it myself
% Rotation
M = C(:,1:3);
% getting RQ decomposition using matlab's QR decomp (doesn't have RQ)
[R,Q] = rq(M);

% so R (upper triangular) is scaled K and Q (orthogonal) is the rotation
K = R ./ R(3,3);
R = Q;

% force the main entries of K to be positive, update R accordingly
Icorrection = [1 0 0; 0 1 0; 0 0 1];
if K(1,1) < 0
    Icorrection(1,1) = -1;
end
if K(2,2) < 0
    Icorrection(2,2) = -1;
end
if K(3,3) < 0
    Icorrection(3,3) = -1;
end
K = K * Icorrection;
R = Icorrection * R;

% sometimes the R we want is negative of the R we find... make sure we
% get the right one by flipping all 3 axes if needed:
ztester = [0 0 1]';
neg = -R*ztester;
if neg(3) < 0
    R = -R;
end

% Camera position
camPos3d = -R'*inv(K)*C(:,4);
camRot3d = R;
omega = mat2rod(camRot3d);
fx = K(1,1); fy = K(2,2); cx = K(1,3); cy = K(2,3); s = K(1,2);
intrinsics =  [omega fx fy cx cy s];
results = [camPos3d' intrinsics];
%}