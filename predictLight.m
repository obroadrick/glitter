% predict light position given 
% known characterization and known camera position

P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
tiledlayout(2,2,"TileSpacing","tight","Padding","tight");

%% read in the image
% from circle on screen:
%impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
%im = rgb2gray(imread(impath));
% from light source not on monitor (xenon!)
%ambient = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Glitter-Ambient.jpg'));
%shined = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Glitter-Point.jpg'));
%ambient = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Poin-Captures-6-14-4_05/2022-06-14T16,05,33Single-Glitter.jpg'));
%shined = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Poin-Captures-6-14-4_05/2022-06-14T16,05,41Single-Glitter.jpg'));
%ambient = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Point-Captures-6-14-4_33/2022-06-14T16,33,02Single-Glitter.jpg'));
%shined = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Point-Captures-6-14-4_33/2022-06-14T16,33,12Single-Glitter.jpg'));
ambient = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Point-Captures-6-16-1_50/Background.jpg'));
shined = rgb2gray(imread('/Users/oliverbroadrick/Desktop/Point-Captures-6-16-1_50/Point2.jpg'));
im = shined-ambient;
% show original image
ax1 = nexttile;
imagesc(im);colormap(gray);
title('original image');

%% get homography from image to canonical glitter coordinates
% we assume here that the homography has already been found 
% in advance of running this script
%tform = matfile(P.tform).tform;
% don't use that old stinky transform, instead use a fresh new one
% points in from addy:
pin = [1127. 5357.; 605. 392.; 6375.  372.; 6074. 5384.];
pin = [1128.2207 5358.38; 604.22656 391.15845; 6374.993 371.94476; 6073.0977 5384.163 ];
pin = [3035.088 4434.6123; 2557.556 388.33606; 7115.98 307.2471; 7094.5967 5029.888];
tform = getTransform(P, pin);
% show transformed image
ax2 = nexttile;
imw = imwarp(im,tform);
imagesc(imw);

%% find spec centroids
imageCentroids = singleImageFindSpecs(im);
out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];
% show centroids
ax3 = nexttile;
imagesc(im); hold on;
plot(imageCentroids(:,1),imageCentroids(:,2),'r+');

%% also show the original max image so that we can compare the centroids
% that we found with the centroids that we characterized from the start
maxImagePath = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/maxImage.jpg';
maxImage = imread(maxImagePath);
knownImageCentroids = matfile(P.imageCentroids).imageCentroids;
ax4 = nexttile;
imagesc(im); hold on;
plot(knownImageCentroids(:,1),knownImageCentroids(:,2),'r+');
linkaxes([ax3 ax4]);

%% match canonical centroids to those in the characterization
knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
[idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids, 'Distance', 'euclidean');
% only consider specs whose match is within .5 millimeters
closeEnough = .5;
specIdxs = idx(dist<closeEnough);

%% trace rays from camera to illuminated specs
% and back out into the world
% get cam position, spec positions, and spec surface normals
%camPos = matfile(P.camPos).camera_in_glitter_coords;%compute new cam pos!
pin = [3035.088 4434.6123; 2557.556 388.33606; 7115.98 307.2471; 7094.5967 5029.888];
camParams = matfile(P.camParams).camParams;
camPos = findCamPos(P, camParams, P.extraOnGlitterPlane, pin);
specPos = knownCanonicalCentroids(specIdxs,:);
allSpecNormals = matfile(P.specNormals).specNormals;
specNormals = allSpecNormals(specIdxs,:);

% compute reflected rays: Ri = Ci − 2(Ci dot Ni)Ni
% where Ci is normalized vector from spec to camera
%       Ni is normalized normal vector
%       Ri is normalized reflected vector
spec2cam = camPos - specPos;
spec2camN = spec2cam ./ vecnorm(spec2cam, 2, 2);
R = spec2camN - 2 * dot(spec2camN, specNormals, 2) .* specNormals;
R = -1 * R;

%% show a diagram for comparison to the measured light position
% get measured light position
% DRAW IT ALL
M = matfile(P.measurements).M;
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
%patch(mx,my,mz,mc);
% table: (made up coords, doesn't matter, mostly for fun)
tx = [-400 -400 600 600];
ty = [-120 -120 -120 -120];
tz = [-250 1000 1000 -250];
tc = ['k'];
%patch(tx,ty,tz,tc);
% show camera as a dot: (dots=cameras)
scatter3(cam(1),cam(2),cam(3),'filled');
%scale for drawing
specNormalsLong = 100 * specNormals;
for ix=1:size(specPos,1)
    % draw normals
    x = [specNormalsLong(ix,1)+specPos(ix,1) specPos(ix,1)]';
    y = [specNormalsLong(ix,2)+specPos(ix,2) specPos(ix,2)]';
    z = [specNormalsLong(ix,3)+specPos(ix,3) specPos(ix,3)]';
    line(x,y,z,'Color','red');
end
for ix=1:size(specPos,1)
    % draw rays from specs to camera
    x = [specPos(ix,1) specPos(ix,1)+spec2cam(ix,1)]';
    y = [specPos(ix,2) specPos(ix,2)+spec2cam(ix,2)]';
    z = [specPos(ix,3) specPos(ix,3)+spec2cam(ix,3)]';
    %line(x,y,z);
end
Rlong = R .* 1000; %one meter in length
for ix=1:size(specPos,1)
    % draw reflected rays
    x = [specPos(ix,1) specPos(ix,1)+Rlong(ix,1)]';
    y = [specPos(ix,2) specPos(ix,2)+Rlong(ix,2)]';
    z = [specPos(ix,3) specPos(ix,3)+Rlong(ix,3)]';
    line(x,y,z);
end
% shown known light source (which we didn't use to ray trace)
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];
%scatter3(lightPos(1), lightPos(2), lightPos(3), 'filled', 'o');
t = linspace(0, 2*pi);
r = 48 * M.PX2MM_X;
x = r*cos(t) + lightPos(1);
y = r*sin(t) + lightPos(2);
z = zeros(size(t))+M.GLIT_TO_MON_PLANES;
%patch(x, y, z, 'b');
% set viewpoint:
view([-110 -30]);
camroll(-80);
daspect([1 1 1]);
