% predict a sparkle pattern given
%      - lighting position
%      - camera position
%      - glitter characterization 
%               - (spec positions and surface normals)
rng(23);
close all;

% get useful paths
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P; 
M = matfile(P.measurements).M;

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];

% get camera position, spec positions, and spec normals
camPos = matfile(P.camPos).camera_in_glitter_coords;
specPos = matfile(P.canonicalCentroids).canonicalCentroids;
specNormals = matfile(P.specNormals).specNormals;

% compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
% where Li is normalized vector from spec to light
%       Ni is normalized normal vector
%       Ri is normalized reflected vector
L = (lightPos - specPos) ./ vecnorm(lightPos - specPos, 2, 2);
%Lrep = repmat(L,size(specNormals,1),1);%repeated for dot products at once
R = L - 2 * dot(L, specNormals, 2) .* specNormals;

% now check how many of the reflected rays pass within a range of 
% the pinhole
range = 10; %in millimeters
% first find where on the plane parallel to the glitter and containing 
% the camera the reflectedRays intersect
% this is as simple as multiplying (extending) the reflected rays
% until their third (z) coordinate is equal to that of the camPos
Rtocam = R .* (camPos(3) ./ R(:,3));
% now the x and y position for each entry can be compared to camera pos
% and those that are sparkling (seen/on) are those that are within the range
seen = sqrt((specPos(:,1)+Rtocam(:,1)-camPos(1)).^2+...
        (specPos(:,2)+Rtocam(:,2)-camPos(2)).^2) < range;


% NOTE: i have traced these light rays in perhaps the reverse of the most
% uesful direction for when i go to set thresholds/etc on the glitter
% receptive field

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




