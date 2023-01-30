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
    if ~isfile([expdir '/' imnames{i} '_16pts.mat'])
        % Use Addy's Python scipt to detect the ArUco markers
        setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
        cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/detectArUcos.py "%s" "%s"', impaths{i}, [expdir '/' imnames{i} '_16pts.mat']);
        system(cmd);
    end
end

%%
% Display images with detected ArUco marker points
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
        end
    end

    % Draw plot
    set(ax,'xticklabel',[],'yticklabel',[])
    title(d(i).name);
    drawnow;
end

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
        continue
    end
    % If the points are not all unique, ignore this image (failure of our
    % detection code)
    if ~(size(unique(arucoPoints{i},'rows'), 1) == size(arucoPoints{i}, 1))
        continue
    end
    for j=1:size(arucoPoints{i},1)
        if ~(arucoPoints{i}(j,1) > 0 && arucoPoints{i}(j,2) > 0)
            continue
        end
        imPts(numPts,:) = arucoPoints{i}(j,:);
        worldPts(numPts,:) = worldPoints{i}(j,:);
        numPts = numPts + 1;
    end
end

%{
%% 
% uUse only unique entries
[imPtsPrime, ia, ic] = unique(imPts,'row');
worldPtsPrime = worldPts(ia,:);
imPts=imPtsPrime;worldPts=worldPtsPrime;
%}

%%
% Get the camera matrix
[C, r] = estimateCameraMatrix(imPts,worldPts);
C = C';

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
counter = 0;
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
            % Show the ArUco point
            plot(pts{i}(ix,1), pts{i}(ix,2),'cX','MarkerSize', 20, 'LineWidth',2);

            % Show the world point reprojected according to our camera
            % matrix
            reproj = reproject(worldPoints{i}(ix,:)',C);
            plot(reproj(1), reproj(2), 'r+', 'MarkerSize', 15, 'LineWidth',2);

            counter = counter + 1;
        end
    end

    % Draw plot
    set(ax,'xticklabel',[],'yticklabel',[])
    title(d(i).name);
    drawnow;
end

%%
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

%%
% Decompose
[K,R,IminusC]= Pdecomp(C);
camPos3d = -IminusC(:,4);
camRot3d = R;
K = K ./ K(3,3);
omega = mat2rod(camRot3d);
fx = K(1,1); fy = K(2,2); cx = K(1,3); cy = K(2,3); s = K(1,2);
intrinsics =  [omega fx fy cx cy s];
results = [camPos3d' intrinsics];

% Save results
save('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults.mat',"results");

function r = reproject(X,C)
    y = C * [X; 1];
    r = y ./ y(3);
end

%{
% At first I began implementing this myself... how silly.
%%
% Solve the linear system to get the camera parameters using these
% correspondences
for i=1:size(imPts,1)
    C(i*3-2,:) = [worldPts(i,1) worldPts(i,2) worldPts(i,3) 1 0 0 0 0 0 0 0 0 zeros(1,i-1) -imPts(i,1) zeros(1,size(imPts,1)-i+1)];
    C(i*3-1,:) = [0 0 0 0 worldPts(i,1) worldPts(i,2) worldPts(i,3) 1 0 0 0 0 zeros(1,i-1) -imPts(i,2) zeros(1,size(imPts,1)-i+1)];
    C(i*3,:) =   [0 0 0 0 0 0 0 0 worldPts(i,1) worldPts(i,2) worldPts(i,3) 1 zeros(1,i-1) -1          zeros(1,size(imPts,1)-i+1)];
    %{
    b(i*3-2,:) = [imPts(i,1)];
    b(i*3-1,:) = [imPts(i,2)];
    b(i*3,:) = [1];
    %}
end
b = zeros(size(C,1),1);
x = C\b;
%}