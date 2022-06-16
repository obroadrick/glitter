% find translation from glitter to camera using a known light 
% source, a picture of sparkling glitter, and a known glitter
% characterization

P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;

% read in image
impath = '......';
im = rgb2gray(imread(impath));

% suppose we know the lighting position in canonical coordinates
lightPos = [100 200 300];

%% find spec centroids in image
pin = [3035.088 4434.6123; 2557.556 388.33606; 7115.98 307.2471; 7094.5967 5029.888];
tform = getTransform(P, pin);
imageCentroids = singleImageFindSpecs(im);
out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];

%% match canonical centroids to those in the characterization
knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
[idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids, 'Distance', 'euclidean');
% only consider specs whose match is within .5 millimeters
closeEnough = .2;
specIdxs = idx(dist<closeEnough);
specPos = knownCanonicalCentroids(specIdxs,:);
allSpecNormals = matfile(P.specNormals).specNormals;
specNormals = allSpecNormals(specIdxs,:);

%% reflect rays from light off specs
% compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
% where Li is normalized vector from spec to light
%       Ni is normalized normal vector
%       Ri is normalized reflected vector
L = (lightPos - specPos) ./ vecnorm(lightPos - specPos, 2, 2);
R = L - 2 * dot(L, specNormals, 2) .* specNormals;

% estimate camera position by minimizing some error
% function
function ret = errFun(c)
    ret = err(c, specPos, R);
end
x0 = [M.GLIT_SIDE/2;M.GLIT_SIDE/2;M.GLIT_SIDE];
camPos = fminsearch(errFun,x0);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DRAW IT ALL FOR DEBUGGING:

% get camera position, spec positions, and spec normals
camPos = matfile(P.camPos).camera_in_glitter_coords;
specPos = matfile(P.canonicalCentroids).canonicalCentroids;
specNormals = matfile(P.specNormals).specNormals;

% draw rig
cam=camPos;
figure;
% glitter square:
gx = [0 M.GLIT_SIDE M.GLIT_SIDE 0]; 
gy = [0 0 M.GLIT_SIDE M.GLIT_SIDE]; 
gz = [0 0 0 0];
gc = ['b'];
patch(gx,gy,gz,gc);hold on;
% monitor:
mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
mc = ['g'];
patch(mx,my,mz,mc);
% table: (made up coords, doesn't matter, mostly for fun)
tx = [-400 -400 600 600];
ty = [-120 -120 -120 -120];
tz = [-250 1000 1000 -250];
tc = ['k'];
patch(tx,ty,tz,tc);
% show camera as a dot: (dots=cameras)
scatter3(cam(1),cam(2),cam(3),'filled');
% show light source as a dot
scatter3(lightPos(1),lightPos(2),lightPos(3),'filled');
% draw all the passed lines
%scale for drawing
specNormals = specNormals .* 50;
%reflectedRaysCamPlane = reflectedRaysCamPlane .* 100;

howmany = 1000;
cxs = randi(size(specNormals,1),howmany,1);
for ix=cxs%size(specNormals,1)
    % draw normals
    x = [specNormals(ix,1)+specPos(ix,1) specPos(ix,1)]';
    y = [specNormals(ix,2)+specPos(ix,2) specPos(ix,2)]';
    z = [specNormals(ix,3)+specPos(ix,3) specPos(ix,3)]';
    line(x,y,z);
end
for ix=seen%size(specNormals,1)
    % draw reflected rays
    %if norm(Rtocam(ix,:)) > 10000
    %    %disp(norm(Rtocam(ix,:)))
    %    continue
    %end
    x = [specPos(ix,1) specPos(ix,1)+Rtocam(ix,1)]';
    y = [specPos(ix,2) specPos(ix,2)+Rtocam(ix,2)]';
    z = [specPos(ix,3) specPos(ix,3)+Rtocam(ix,3)]';
    line(x,y,z);
end
% set viewpoint:
view([-110 -30]);
camroll(-80);
daspect([1 1 1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% now show just a black glitter square with the bright specs that expected
% to sparkle turned white
figure;
t = tiledlayout(1,2, 'TileSpacing','tight', 'Padding','tight');
num = 16;
t.Padding = 'compact';
t.TileSpacing = 'compact';
ax1 = nexttile;
gx = [0 M.XRES M.XRES 0]; 
gy = [0 0 M.YRES M.YRES]; 
gz = [0 0 0 0];
gc = ['k'];
patch(gx,gy,gz,gc);hold on;
sparkles = specPos(seen,:);
%corners = [0 0 0; 0 M.GLIT_SIDE 0; M.GLIT_SIDE M.GLIT_SIDE 0; M.GLIT_SIDE 0 0];
pout = [0 0; 0 305; 305 305; 305 0];%this is true to glitter coords for the project
markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                    M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                    (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                    (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
corners = pout + markeradjustments;
% get inverse transform to map these sparkles to where they will be in
% the image
tform = invert(matfile(P.tform).tform);
sparklesOut = transformPointsForward(tform, [sparkles(:,1) sparkles(:,2)]);
cornersOut = transformPointsForward(tform, [corners(:,1) corners(:,2)]);
scatter(sparklesOut(:,1), sparklesOut(:,2), 12, 'filled', 'white');
scatter(cornersOut(:,1), cornersOut(:,2), 12, 'filled', 'red');
set(gca, 'YDir','reverse');axis equal;
% show next to the corresponding test image
% read in image
name = '2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
im = imread([P.characterizationTest name]);
ax2 = nexttile;
imagesc(im);hold on;
%show points Addy detected on fiducial markers
markers = [1234. 4976.; ...
                1025.   64.;...
                6528.  495.;...
                6197. 5009.];
scatter(markers(:,1), markers(:,2), 12, 'filled', 'red');
linkaxes([ax1 ax2]);
ylim([-500 6000]);axis equal;




