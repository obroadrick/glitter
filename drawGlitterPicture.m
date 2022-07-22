clear;
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
M = matfile(P.measurements).M;
knownCamPos = matfile(P.camPos).camPos;
knownCamPos = [220,370,700]
monitorCoords = [3000 1500];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];
figure;
% glitter square:
gx = [0 M.GLIT_WIDTH M.GLIT_WIDTH 0]; 
gy = [0 0 M.GLIT_HEIGHT M.GLIT_HEIGHT];
gz = [0 0 0 0];
gc = ['b'];
legendItems = [];
legendItems(1) = patch(gx,gy,gz,gc,'DisplayName', 'Glitter','FaceAlpha',0);hold on;
% monitor:
mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
mc = ['g'];
legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor','FaceAlpha',0);
% table: (made up coords, doesn't matter, mostly for fun)
tx = [-400 -400 600 600];
ty = [-120 -120 -120 -120];
tz = [-250 1000 1000 -250];
tc = ['k'];
%legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table','FaceAlpha',0);
% shown known ground truth camera position
%legendItems(size(legendItems,2)+1) = scatter3(knownCamPos(1),knownCamPos(2),knownCamPos(3),100,'black','filled','o','DisplayName','True Camera');
T = matfile(P.camPos).camPos;
T =knownCamPos;

R = matfile(P.camRot).camRot;
pose = rigid3d(R,T);hold on;
camObj = plotCamera('AbsolutePose',pose,'Opacity',0,'Size',50,'Color','black');
% show light source as a dot
legendItems(size(legendItems,2)+1) = scatter3(lightPos(1),lightPos(2),lightPos(3),[],'black','filled','DisplayName','Light');
%draw all lines red to start
numSpecs = 10;
for ix=1:numSpecs
    specPos(ix,:) = [rand()*M.GLIT_WIDTH rand()*M.GLIT_HEIGHT 0];
    mu = 0;
    sigma = 6;
    noise = [normrnd(mu,sigma) normrnd(mu,sigma) normrnd(mu,sigma)];
    R(ix,:) = knownCamPos+noise - specPos(ix,:);
end
Rlong = R .* .92;
for ix=1:size(Rlong,1)
    % draw reflected rays
    x = [specPos(ix,1) specPos(ix,1)+Rlong(ix,1)]';
    y = [specPos(ix,2) specPos(ix,2)+Rlong(ix,2)]';
    z = [specPos(ix,3) specPos(ix,3)+Rlong(ix,3)]';
    color = [1 0 0];
    line(x,y,z,'Color','black');
end
for ix=1:size(specPos,1)
    % draw reflected rays
    x = [specPos(ix,1) lightPos(1)]';
    y = [specPos(ix,2) lightPos(2)]';
    z = [specPos(ix,3) lightPos(3)]';
    color = [1 0 0];
    line(x,y,z,'Color','black');
end
%draw sparkles
scatter3(specPos(:,1),specPos(:,2),specPos(:,3),[],'black','*')
axis vis3d
axis off