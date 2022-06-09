% predict a sparkle pattern given
%      - lighting position
%      - camera position
%      - glitter characterization 
%               - (spec positions and surface normals)

% get useful paths
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P; 

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y 0];

% get camera position, spec positions, and spec normals
camPos = matfile(P.camPos).camPos;
specPos = matfile(P.canonicalCentroids).canonicalCentroids;
specNormals = matfile(P.specNormals).specNormals;

% 
