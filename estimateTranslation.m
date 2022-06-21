% find translation from glitter to camera using a known light 
% source, a picture of sparkling glitter, and a known glitter
% characterization

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
[idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids, 'Distance', 'euclidean');
% only consider specs whose match is within .xx millimeters
closeEnough = .3;
specIdxs = idx(dist<closeEnough);
specPos = knownCanonicalCentroids(specIdxs,:);
allSpecNormals = matfile(P.specNormals).specNormals;
specNormals = allSpecNormals(specIdxs,:);
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
% compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
% where Li is normalized vector from spec to light
%       Ni is normalized normal vector
%       Ri is normalized reflected vector
L = (lightPos - specPos) ./ vecnorm(lightPos - specPos, 2, 2);
R = L - 2 * dot(L, specNormals, 2) .* specNormals;
R = -1.*R;

% estimate camera position by minimizing some error function
errFun = @(c) err(c, specPos, R);
%x0 = [M.GLIT_SIDE/2;M.GLIT_SIDE/2;M.GLIT_SIDE];
%x0 = [0;0;0];
x0 = [M.GLIT_SIDE;M.GLIT_SIDE;2*M.GLIT_SIDE];
options = optimset('PlotFcns',@optimplotfval);
camPosEst = fminsearch(errFun,x0,options)';
disp(camPosEst);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAW IT ALL FOR DEBUGGING:

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
for ix=1:size(R,1)
    % draw reflected rays
    x = [specPos(ix,1) specPos(ix,1)+Rtocam(ix,1)]';
    y = [specPos(ix,2) specPos(ix,2)+Rtocam(ix,2)]';
    z = [specPos(ix,3) specPos(ix,3)+Rtocam(ix,3)]';
    %distance form camera estimate
    green = [0 1 0];
    red = [1 0 0];
    c = min(1, distNormalized(ix));
    color = green + c*(red-green);
    %color = [1 c 1];
    line(x,y,z,'Color',color);
end
% set viewpoint:
view([-110 -30]);
camroll(-80);
daspect([1 1 1]);
legend(legendItems);