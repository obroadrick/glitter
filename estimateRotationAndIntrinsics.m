% estimate rotation and focal length
% this is a very simple version of camera calibration
% in which a camera point p in homogenous coordinates
% for a world point P in world coordinates is given by
% p = K(PR+T) where T is the translation we already solved for
% and R is the rotation from world to camera coordinates we solve for here
% and K is the camera instrinsics matrix which we solve for here as well.
% we parameterize R by 3 components, rodrigues parameters
% we parameterize K by just a single component f so that we have 
% K = [f 0 w/2; 0 f h/2; 0 0 1]; where w and h are the width 
% and height in pixels of the image

P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
M = matfile(P.measurements).M;

% read in image
impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
im = rgb2gray(imread(impath));

%% get lighting position in canonical coords form lighting position in
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
% draw vector map where each vector goes from a spec to its nearest spec
% neighbor
%figure;
%quiver(canonicalCentroids(:,1), canonicalCentroids(:,2), knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1), knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2),'LineWidth',2);
%x = knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1);
%y = knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2);
%figure;
%histogram(atan2(y, x));title('histogram of vector directions');
%xlabel('direction in radians');

%% get spec normals and brightness
allSpecNormals = matfile(P.specNormals).specNormals;
specNormals = zeros(size(idx,1),K,3);
for ix=1:size(idx,1)
    specNormals(ix,:,:) = allSpecNormals(idx(ix,:),:);
end
allMaxBrightness = matfile(P.maxBrightness).maxBrightness;
maxBrightness = zeros(size(idx,1),K);
for ix=1:size(idx,1)
    maxBrightness(ix,:) = allMaxBrightness(idx(ix,:));
end

%% now make an estimate of the rotation and instrinsics matrices for this
% camera calibration 

% known points in world coordinates
P = bestSpecPos; % the characterized, canonical spec positions that correspond to the sparkling specs in the image
p = imageCentroids;
w = M.XRES;
h = M.YRES;
T = reshape(camPosEst,3,1);
%             errRK(f,    w, h, r1,   r2,   r3,   p, P, T)
errFun = @(x) errRK(x(1), w, h, x(2), x(3), x(4), p, P, T);
x0 = [1000 1 1 1]';%[100; 2; 1; 1;];
options = optimset('PlotFcns',@optimplotfval);
xf = fminsearch(errFun, x0, options);
f = xf(1);
r1 = xf(2);
r2 = xf(3);
r3 = xf(4);
R = rodrigues(r1,r2,r3);

%% draw the scene with camera and its frustrum
figure;
pose = rigid3d(R,T');
camObj = plotCamera('AbsolutePose',pose,'Opacity',0,'Size',35);
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
