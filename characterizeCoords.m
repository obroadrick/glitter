clear;close all;
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
means = matfile([datap 'lightingmeans_2022_06_02.mat']).means;
C = matfile([datap 'centroids_2022_06_01.mat']).C;
M = matfile([datap 'measurements.mat']).M;

%% spec to lightsource vectors in glitter coords
% first: just get the first such vector to see if it
% is reasonable

% map the gaussian means to lighting positions
% note that this can (and should and will) be recast
% as matrix operations and done for all of the points
% at once...
howmany = 1000;
cxs = randi(size(C,1),howmany,1);
x = (-M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - (M.FIRST_INDEX_X + M.PX2MM_X .* (means(1,cxs)-1)))'; 
y = (-M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - (M.FIRST_INDEX_Y + M.PX2MM_Y .* (means(2,cxs)-1)))'; 
z = zeros(howmany,1) + M.GLIT_TO_MON_PLANES;
lightPos = [x y z];
% point correspondences from Addy
pin = [1571. 5129.;
 1418.  339.;
 6863.  549.;
  6276. 5220.];
%order: bottom-left, top-left, top-right, bottom-right
pout = [0 305; 0 0; 305 0; 305 305];
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
disp(origin);
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
% draw the glitter rig with this line showing
% drawRig(M, [],[]);%reshape(lightPos, 1,1,3), reshape(specPos, 1,1,3));
%drawRig(M, lightPos, specPos);


%%%%EXTRA
% vector from spec to light
%frstLightToSpec = lightPos + specPos;
% normalize
%frstLightToSpec = frstLightToSpec / norm(frstLightToSpec);
%camCal = [40.9412 6.2035 625.3044];

