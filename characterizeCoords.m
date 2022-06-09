clear;close all;
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
means = matfile([datap 'lightingmeans_2022_06_02.mat']).means;
C = matfile([datap 'image_centroids_2022_06_01.mat']).C;
M = matfile([datap 'measurements.mat']).M;
cam = matfile([datap 'camera_in_glitter_coords_06_08_2022.mat']).camera_in_glitter_coords;
% seed the rng
seed = 125;
rng(seed);
%% spec to lightsource vectors in glitter coords
% first: just get the first such vector to see if it
% is reasonable

% map the gaussian means to lighting positions
% note that this can (and should and will) be recast
% as matrix operations and done for all of the points
% at once...
%RANDOM
%howmany = 5000;
howmany = size(C,1);
%rng(125);
%cxs = randi(size(C,1),howmany,1);
%ALL
cxs = [1:size(C,1)];
x = (-M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - (M.FIRST_INDEX_X + (M.PX2MM_X * M.INDEX_TO_PX .* (means(1,cxs)-1))))'; 
y = (-M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - (M.FIRST_INDEX_Y + (M.PX2MM_Y * M.INDEX_TO_PX .* (means(2,cxs)-1))))'; 
z = zeros(howmany,1) + M.GLIT_TO_MON_PLANES;
lightPos = [x y z];
% point correspondences from Addy from fiducial markers (all lower left
% corners)
pin = [1571. 5129.;
 1418.  339.;
 6863.  549.;
  6276. 5220.];
%order: bottom-left, top-left, top-right, bottom-right
%pout = [0 305; 0 0; 305 0; 305 305];%this is true to image coords for 2d showing
pout = [0 0; 0 305; 305 305; 305 0];%this is true to glitter coords for the project
markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                    M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                    (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                    (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
pout = pout + markeradjustments;
% homography
tform = fitgeotrans(pin, pout, 'projective');
% show max image and original position of glitter specs
m = imread('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/imgs/june1/max_june_1.jpg');
clf;
% original:
t = tiledlayout(1,2);
t.Padding = 'compact';
t.TileSpacing = 'compact';
nexttile;
imagesc(m);hold on;
colormap(gray);
msize = int32(.005*(size(m,1)+size(m,2))/2);
scatter([C(cxs,1)], [C(cxs,2)], msize,'filled');
scatter(pin(:,1),pin(:,2),msize,'red','filled');
drawnow;
% warped: apply homography and show
nexttile;
out = transformPointsForward(tform, [C(cxs,1) C(cxs,2)]);
% find new coordinate of 0,0 after transform to set viewpoint for image
origin = transformPointsForward(tform, [0 0]);
% add this origin to all the points for us to show them on top of image
%outView = imref2d([size(m,1)+origin(1),size(m,2)+origin(2)]);
%outputImage = imwarp(m,tform,'OutputView',outView)
outputImage = imwarp(m,tform);
imagesc(outputImage);hold on;title('here');
%show where the four known points are
%imagesc(m);hold on;title('here');
%scatter(pin(:,1),pin(:,2),'filled');
%scatter(out(:,1), out(:,2), 'filled');
drawnow;
specPos = [out(:,1) out(:,2) zeros(size(out,1),1)];

% vector from spec to light
spec2light = lightPos - specPos;

% normalize
spec2light = spec2light ./ vecnorm(spec2light, 2, 1);

% vector from spec to camera
spec2cam = cam - specPos;

% normalize
spec2cam = spec2cam ./ vecnorm(spec2cam, 2, 1);

% spec surface normals 
spec_normals = spec2light + spec2cam; %just add since they are normalized
spec_normals = spec_normals ./ vecnorm(spec_normals, 2, 1);
save([datap 'spec_normals_' datestr(now, 'mm_dd_yyyy')], "spec_normals");

%% histograms of surface normal components in each direction (x,y)
figure;
t = tiledlayout(1,2);
t.Padding = 'compact';
t.TileSpacing = 'compact';
ax1 = nexttile(1);
histogram(pi/2-acos(spec_normals(:,1)));
xline(mean(pi/2-acos(spec_normals(:,1))));
title('Horizontal Components of Spec Surface Normals');
xlabel('Radians');
ax2 = nexttile(2);
histogram(asin(spec_normals(:,2)));
xline(mean(asin(spec_normals(:,2))));
title('Vertical Components of Spec Surface Normals');
xlabel('Radians');
linkaxes([ax1 ax2], 'xy');

%% pretty picture of it all
% draw the glitter rig with these lines showing
number_to_draw = 10;
draw_idxs = randi(size(C,1),number_to_draw,1);
drawRig(M, lightPos(draw_idxs,:), specPos(draw_idxs,:), cam);


