% show intensity vs distance to pinhole
% known light source, known camera position, known characterization

% read in image
impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
im = rgb2gray(imread(impath));

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];

% get camera position
%camPos = matfile(P.camPos).camera_in_glitter_coords;
camParams = matfile(P.camParams).camParams;
pin = [1217.34838867 5145.87841797; 1005.55084  295.4278; 6501.5874  490.0575; 6501.952 5363.594];
[camPos, camRotation] = findCamPos(P, camParams, P.onGlitterPlane, pin);

%% find spec centroids in image
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

% find distance from pinhole to ray
%Rtocam = R .* (camPos(3) ./ R(:,3)); %doing so just with parallel plane
distsFromPinhole = [];
brightnesses = [];
pinHole = camPos;
pinHoleNormal = inv(camRotation)*[0;0;-1];
for ix=1:size(R,1)
end


