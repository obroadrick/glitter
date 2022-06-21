% find translation from glitter to camera using a known light 
% source, a picture of sparkling glitter, and a known glitter
% characterization
clear;
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
M = matfile(P.measurements).M;

% read in image
impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
im = rgb2gray(imread(impath));

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];

%% find spec centroids in image
%pin = [1217.34838867 5145.87841797; 1005.55084  295.4278; 6501.5874  490.0575; 6501.952 5363.594];
pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y 
tform = getTransform(P, pin);
imageCentroids = singleImageFindSpecs(im);
out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];


%% match canonical centroids to those in the characterization
knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
K = 10;
[idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids,...
                             'K', K, 'Distance', 'euclidean');
% only consider specs whose match is within .xx millimeters
%closeEnough = .3;
%specIdxs = idx(dist<closeEnough);
%specPos = knownCanonicalCentroids(idx,:);
specPos = zeros(size(idx,1),K,3);
for ix=1:size(idx,1)
    specPos(ix,:,:) = knownCanonicalCentroids(idx(ix,:),:);
end

allSpecNormals = matfile(P.specNormals).specNormals;
specNormals = zeros(size(idx,1),K,3);
for ix=1:size(idx,1)
    specNormals(ix,:,:) = allSpecNormals(idx(ix,:),:);
end


% also get the brightness of those specs
brightness = [];
%imageCentroids = imageCentroids(dist<closeEnough,:);
for ix=1:size(imageCentroids,1)
    brightness(ix) = interp2(im, imageCentroids(ix,1), imageCentroids(ix,2));
end
%% also show the original max image so that we can compare the centroids
% that we found with the centroids that we characterized from the start
tiledlayout(1,1);colormap(gray);
ax3 = nexttile;
imagesc(im); hold on;
% now show the canonical spec centroids (mapped onto this image coordinate
% system using the inverse homography) to show them 
tforminv = invert(tform);
characterizedCentroidsOnThisImage = transformPointsForward(tforminv, [knownCanonicalCentroids(:,1) knownCanonicalCentroids(:,2)]);
plot(characterizedCentroidsOnThisImage(:,1),characterizedCentroidsOnThisImage(:,2),'r+');
plot(imageCentroids(:,1),imageCentroids(:,2),'bx');
maxImagePath = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/maxImage.jpg';
maxImage = imread(maxImagePath);
knownImageCentroids = matfile(P.imageCentroids).imageCentroids;
%ax4 = nexttile;
%imagesc(maxImage); hold on;
%plot(knownImageCentroids(:,1),knownImageCentroids(:,2),'r+');
%linkaxes([ax3 ax4]);

%% reflect rays from light off specs
%reflect the ten nearest specs for each spec found
R = zeros(size(specPos));
for ix=1:K
    % compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
    % where Li is normalized vector from spec to light
    %       Ni is normalized normal vector
    %       Ri is normalized reflected vector
    L = (lightPos - reshape(specPos(:,ix,:),size(specPos,1),size(specPos,3))) ./ vecnorm(lightPos - reshape(specPos(:,ix,:),size(specPos,1),size(specPos,3)), 2, 2);
    R(:,ix,:) = L - 2 * dot(L, reshape(specNormals(:,ix,:),size(specNormals,1),size(specNormals,3)), 2) .* reshape(specNormals(:,ix,:),size(specNormals,1),size(specNormals,3));
    R(:,ix,:) = -1.*R(:,ix,:);
end

% estimate camera position by minimizing some error function
errFun = @(c) err(c, specPos, R);
%x0 = [M.GLIT_SIDE/2;M.GLIT_SIDE/2;M.GLIT_SIDE];
%x0 = [0;0;0];
%x0 = [M.GLIT_SIDE;M.GLIT_SIDE;2*M.GLIT_SIDE];
%options = optimset('PlotFcns',@optimplotfval);
%camPosEst = fminsearch(errFun,x0,options)';
%disp(camPosEst);

% find a good translation estimate using a RANSAC approach
rng(314159);
inlierThreshold = 10; % (mm) a reflected ray is an inlier
                      % with respect to a hypothesized camera position
                      % if it pases within 10 millimeters of that 
                      % camera position

                      % draw rig
figure;
% glitter square:
gx = [0 M.GLIT_SIDE M.GLIT_SIDE 0]; 
gy = [0 0 M.GLIT_SIDE M.GLIT_SIDE]; 
gz = [0 0 0 0];
gc = ['b'];
legendItems = [];
legendItems(1) = patch(gx,gy,gz,gc,'DisplayName', 'Glitter');hold on;
% monitor:
mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
mc = ['g'];
legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
% table: (made up coords, doesn't matter, mostly for fun)
tx = [-400 -400 600 600];
ty = [-120 -120 -120 -120];
tz = [-250 1000 1000 -250];
tc = ['k'];
legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
% shown known ground truth camera position
knownCamPos = matfile(P.camPos).camera_in_glitter_coords;
legendItems(size(legendItems,2)+1) = scatter3(knownCamPos(1),knownCamPos(2),knownCamPos(3),100,'blue','filled','o','DisplayName','True Camera');
% show light source as a dot
legendItems(size(legendItems,2)+1) = scatter3(lightPos(1),lightPos(2),lightPos(3),'filled','DisplayName','Light');

numInliers = [];
legPos = size(legendItems,2)+1;
camroll(-80);
mostInliersSpecPos = [];
mostInliersR = [];
for counter=1:100
    % hypothesize a possible pair of inliers
    idxs = randi(size(R,1),1,2);
    k = 1; %when hypothesizing, just take the neareast neighbor specs
    if nearestDistLines(specPos(idxs,k,:), R(idxs,k,:)) > 2 * inlierThreshold
        continue
    end
    % find the corresponding candidate camera position
    points = specPos(idxs,k,:);
    directions = R(idxs,k,:);
    candidate = pointBetweenLines(points, directions);
    % sometimes the lines cross near each other behind the glitter plane...
    if candidate(3) < 0
        continue
    end
    % show camera estimated position as a dot: (dots=cameras)
    cam=candidate;
    legendItems(legPos) = scatter3(cam(1),cam(2),cam(3),25,'red','o','filled','DisplayName','Hypothesized Camera');
    % check how many of the other rays are inliers for this hypothesized
    % camera position
    dists = [];
    for k=1:size(R,2)%size(R,2) is also just K from knnsearch above
        for ix=1:size(R,1)
            dists(ix,k) = distPointToLine(candidate, reshape(specPos(ix,k,:),1,3), reshape(R(ix,k,:),1,3));
        end
    end
    % a single spec doesn't get multiple reflected rays, so we just
    % consider the best reflected ray, assuming that that one is the
    % correct one for this model probably
    dists = min(dists, [], 2);
    numInliers(counter) = sum(dists<=inlierThreshold);
    inliersSpecPos = specPos(dists<=inlierThreshold,:);
    inliersR = R(dists<=inlierThreshold,:);
    inliersLogicalIdxs = dists<=inlierThreshold;
    if size(inliersR) > size(mostInliersR)
        mostInliersR = inliersR;
        mostInliersSpecPos = inliersSpecPos;
        mostInliersLogicalIdxs = inliersLogicalIdxs;
    end
    % now show the two lines
    for i=1:size(idxs,1)
        ix = idxs(i);
        % draw reflected rays
        x = [specPos(ix,1) specPos(ix,1)+Rtocam(ix,1)]';
        y = [specPos(ix,2) specPos(ix,2)+Rtocam(ix,2)]';
        z = [specPos(ix,3) specPos(ix,3)+Rtocam(ix,3)]';
        color = [1 0 0];
        line(x,y,z,'Color',color);
    end
    % and their inliers as well
    inliersR = inliersR * 1000;
    for ix=1:size(inliersR,1)
        % draw reflected rays
        x = [inliersSpecPos(ix,1) inliersSpecPos(ix,1)+inliersR(ix,1)]';
        y = [inliersSpecPos(ix,2) inliersSpecPos(ix,2)+inliersR(ix,2)]';
        z = [inliersSpecPos(ix,3) inliersSpecPos(ix,3)+inliersR(ix,3)]';
        color = [0 1 0];
        line(x,y,z,'Color',color);
    end
    view([-110 -30]);
    %camroll(-80);
    daspect([1 1 1]);
    legend(legendItems);
    drawnow;
    curNumInliers = numInliers(size(numInliers,2));
    title(['Number of inliers: ' num2str(curNumInliers) ' (max so far: ' num2str(max(numInliers)) ')']);
    ot (.5);
    % now go back over and draw them all back to red
    for i=1:size(idxs,1)
        ix = idxs(i);
        % draw reflected rays
        x = [specPos(ix,1) specPos(ix,1)+Rtocam(ix,1)]';
        y = [specPos(ix,2) specPos(ix,2)+Rtocam(ix,2)]';
        z = [specPos(ix,3) specPos(ix,3)+Rtocam(ix,3)]';
        line(x,y,z,'Color',red);
    end
    % and their inliers as well
    for ix=1:size(inliersR,1)
        % draw reflected rays
        x = [inliersSpecPos(ix,1) inliersSpecPos(ix,1)+inliersR(ix,1)]';
        y = [inliersSpecPos(ix,2) inliersSpecPos(ix,2)+inliersR(ix,2)]';
        z = [inliersSpecPos(ix,3) inliersSpecPos(ix,3)+inliersR(ix,3)]';
        line(x,y,z,'Color',red);
    end
end
figure;
histogram(numInliers);

% now just for the model with the most inliers, we build up a 
% camera position estimate
% estimate camera position by minimizing some error function
errFun = @(c) err(c, mostInliersSpecPos, mostInliersR);
%x0 = [M.GLIT_SIDE/2;M.GLIT_SIDE/2;M.GLIT_SIDE];
%x0 = [0;0;0];
x0 = [M.GLIT_SIDE;M.GLIT_SIDE;2*M.GLIT_SIDE];
options = optimset('PlotFcns',@optimplotfval);
camPosEst = fminsearch(errFun,x0,options)';
disp(camPosEst);

% also compute dists to the known camera location
trueDists = [];
for ix=1:size(R,1)
    trueDists(ix) = distPointToLine(knownCamPos, specPos(ix,:), R(ix,:));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% show camera estimated position as a dot: (dots=cameras)
cam=camPosEst;
legendItems(size(legendItems,2)+1) = scatter3(cam(1),cam(2),cam(3),100,'red','o','filled','DisplayName','Estimated Camera');
disp(cam);
% draw rig
figure;
% glitter square:
gx = [0 M.GLIT_SIDE M.GLIT_SIDE 0]; 
gy = [0 0 M.GLIT_SIDE M.GLIT_SIDE]; 
gz = [0 0 0 0];
gc = ['b'];
legendItems = [];
legendItems(1) = patch(gx,gy,gz,gc,'DisplayName', 'Glitter');hold on;
% monitor:
mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
mc = ['g'];
legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
% table: (made up coords, doesn't matter, mostly for fun)
tx = [-400 -400 600 600];
ty = [-120 -120 -120 -120];
tz = [-250 1000 1000 -250];
tc = ['k'];
legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
% show camera as a dot: (dots=cameras)
cam=camPosEst;
legendItems(size(legendItems,2)+1) = scatter3(cam(1),cam(2),cam(3),100,'red','o','filled','DisplayName','Estimated Camera');
disp(cam);
% shown known ground truth camera position
knownCamPos = matfile(P.camPos).camera_in_glitter_coords;
disp(knownCamPos);
disp('difference (error):');
disp(norm(cam - knownCamPos));
legendItems(size(legendItems,2)+1) = scatter3(knownCamPos(1),knownCamPos(2),knownCamPos(3),100,'blue','filled','o','DisplayName','True Camera');
% show light source as a dot
legendItems(size(legendItems,2)+1) = scatter3(lightPos(1),lightPos(2),lightPos(3),'filled','DisplayName','Light');
% draw all the passed lines
%scale for drawing
specNormals = specNormals .* 50;
Rtocam = R*1000;
%reflectedRaysCamPlane = reflectedRaysCamPlane .* 100;
for ix=1:size(R,1)
    % draw normals
    x = [specNormals(ix,1)+specPos(ix,1) specPos(ix,1)]';
    y = [specNormals(ix,2)+specPos(ix,2) specPos(ix,2)]';
    z = [specNormals(ix,3)+specPos(ix,3) specPos(ix,3)]';
    line(x,y,z);
end
% get normalized spec distances for drawing
distNormalized = dist / closeEnough;% color code for
brightnessNormalized = double(brightness) / 255.0;% color code for
for ix=1:size(R,1)
    % draw reflected rays
    x = [specPos(ix,1) specPos(ix,1)+Rtocam(ix,1)]';
    y = [specPos(ix,2) specPos(ix,2)+Rtocam(ix,2)]';
    z = [specPos(ix,3) specPos(ix,3)+Rtocam(ix,3)]';
    %distance form camera estimate
    green = [0 1 0];
    red = [1 0 0];
    %c = min(1, distNormalized(ix)); %color code by spec closeness
    c = 1.0*min(1, brightnessNormalized(ix)); %color code by spec brightness
    disp(c);
    color = green + c*(red-green);
    %color = [1 c 1];
    line(x,y,z,'Color',color);
end
title('green rays are from a brighter sparkle, red from a dimmer one');
% set viewpoint:
view([-110 -30]);
camroll(-80);
daspect([1 1 1]);
legend(legendItems);

%
%%%
%%%
%%%%%
%%%%%%%%
%%%%%%%%% now plot brightness vs distance to true pinhole
%%%%%%    
%%%%%
%%%
%
figure;
scatter(trueDists, brightnessNormalized);
xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
title('Intensity vs Dist to Pinhole: All reflected rays');
% also do so for just the inliers of the final estimate
trueDistsInliersOnly = zeros(size(mostInliersR,1),1);
for ix=1:size(mostInliersR,1)
    trueDistsInliersOnly(ix) = distPointToLine(knownCamPos, mostInliersSpecPos(ix,:), mostInliersR(ix,:));
end
brightnessNormalizedInliersOnly = brightnessNormalized(mostInliersLogicalIdxs);
figure;
scatter(trueDistsInliersOnly, brightnessNormalizedInliersOnly);
xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
title('Intensity vs Dist to Pinhole: Inliers Only');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% functions
function d = nearestDistLines(points, directions)
    % normal for the parallel planes each containing one of the lines
    n = cross(directions(1,:),directions(2,:));
    % now get a point from each of the planes (from the lines)
    p1 = points(1,:);
    p2 = points(2,:);
    % a vector from one of these points to the other can then
    % be projected onto the plane normal to get distance
    % between the planes
    v = p1 - p2;
    proj_n_v = dot(v,n,2) / norm(n)^2 .* n;
    d = norm(proj_n_v);
end
function p = pointBetweenLines(points, directions)
    p1 = points(1,:)';
    p2 = points(2,:)';
    d1 = directions(1,:)';
    d2 = directions(2,:)';
    A = [dot(p1,d1) -1*dot(p1,d2);...
         dot(p2,d1) -1*dot(p2,d2)];
    B = [-1*dot(p1-p2,p1);...
         -1*dot(p1-p2,p2)]; 
    x = linsolve(A,B);
    s = x(1);
    t = x(2);
    Q = p1 + d1.*t;
    R = p2 + d2.*s;
    p = (Q+R)./2;
    p = p';
end
function d = distPointToLine(point, pointOnLine, direction)
    a = pointOnLine';% point on the line
    n = (direction./norm(direction))';% unit vector in direction of line
    p = point';% point whose distance is being computed
    d = norm((p-a)-(dot((p-a),n,1)*n));
end