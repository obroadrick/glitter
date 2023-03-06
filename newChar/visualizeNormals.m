% visualize computed surface normals
clear; close all;

% get useful paths
%P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
P = getMar4charPaths();
GLIT_TO_MON_PLANES = 424;
GLIT_TO_MON_EDGES_X = 178;
GLIT_TO_MON_EDGES_Y = 88.8 + 77.1 + 8*18.25;
M = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,GLIT_TO_MON_EDGES_Y);
%et lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y 0];

% get camera position, spec positions, and spec normals
camPos = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/camPosSkew.mat').camPos;
specPos = matfile(P.canonicalCentroids).canonicalCentroids;
specNormals = matfile(P.specNormals).specNormals;

% draw rig
cam=camPos;
figure;
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
%scatter3(cam(1),cam(2),cam(3),'filled');
% draw all the passed lines
specNormals = specNormals .* 50;
cxs = randi(size(specNormals,1),3000,1);
for ix=cxs%size(specNormals,1)
    % show line from light source to glitter spec:
    x = [specNormals(ix,1)+specPos(ix,1) specPos(ix,1)]';
    y = [specNormals(ix,2)+specPos(ix,2) specPos(ix,2)]';
    z = [specNormals(ix,3)+specPos(ix,3) specPos(ix,3)]';
    line(x,y,z);
end
% set viewpoint:
view([-110 -30]);
camroll(-80);
daspect([1 1 1]);