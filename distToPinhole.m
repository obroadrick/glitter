% show intensity vs distance to pinhole
% known light source, known camera position, known characterization

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];

% get camera position
camPos = matfile(P.camPos).camera_in_glitter_coords;

% get known characterization
specPos = matfile(P.canonicalCentroids).canonicalCentroids;
specNormals = matfile(P.specNormals).specNormals;

% trace rays from light source to specs to camera
L = (lightPos - specPos) ./ vecnorm(lightPos - specPos, 2, 2);
R = L - 2 * dot(L, specNormals, 2) .* specNormals;

% find distance from pinhole to ray 
Rtocam = R .* (camPos(3) ./ R(:,3)); %doing so just with parallel plane


