% predict a sparkle pattern given
%      - lighting position
%      - camera position
%      - glitter characterization 
%               - (spec positions and surface normals)
clear;

% get useful paths
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P; 
M = matfile(P.measurements).M;

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y 0];

% get camera position, spec positions, and spec normals
camPos = matfile(P.camPos).camera_in_glitter_coords;
specPos = matfile(P.canonicalCentroids).canonicalCentroids;
specNormals = matfile(P.specNormals).specNormals;

% compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
lightPosRepd = repmat(lightPos,size(specNormals,1),1);
reflectedRays = -lightPos - 2 * dot(lightPosRepd, specNormals, 2) .* specNormals;

% now check how many of the reflected rays pass within a range of 
% the pinhole
range = 5; %in millimeters
% first find where on the plane parallel to the glitter and containing 
% the camera the reflectedRays intersect
% this is as simple as multiplying (extending) the reflected rays
% until their third (z) coordinate is equal to that of the camPos


