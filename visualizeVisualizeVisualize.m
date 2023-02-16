% visualize (in the approximate context of the whole set-up)
% where all these camera position estimates are at...

% hope to debug, planning to cry
M=matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/measurements.mat').M;
figure;
hold on;
axis vis3d;
% glitter square:
gx = [0 M.GLIT_WIDTH M.GLIT_WIDTH 0];
gy = [0 0 M.GLIT_HEIGHT M.GLIT_HEIGHT];
gz = [0 0 0 0];
gc = ['b'];
legendItems = [];
legendItems(1) = patch(gx,gy,gz,gc,'DisplayName', 'Glitter');hold on;
% monitor:
mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
mc = [.2 .2 .2];
legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
% table: (made up coords, doesn't matter, mostly for fun)
tx = [-200 -200 600 600];
ty = [-120 -120 -120 -120];
tz = [-100 700 700 -100];
tc = ['k'];
patch(tx,ty,tz,tc,'DisplayName','Monitor');
hold on;
%{
% now draw the checkboard x and y axes on the glitter plane
plot3([origin_in_glitter_coords(1) xfar_in_glitter_coords(1)],...
    [origin_in_glitter_coords(2) xfar_in_glitter_coords(2)],...
    [origin_in_glitter_coords(3) xfar_in_glitter_coords(3)],'Color','c');
plot3([origin_in_glitter_coords(1) yfar_in_glitter_coords(1)],...
    [origin_in_glitter_coords(2) yfar_in_glitter_coords(2)],...
    [origin_in_glitter_coords(3) yfar_in_glitter_coords(3)],'Color','c');    
% draw canonical coordinate system:
hold on;
%locations
lx = [0 0 0];
ly = [0 0 0];
lz = [0 0 0];
%basis vectors (scaled for viewing)
ex = [100 0 0]; 
ey = [0 100 0]; 
ez = [0 0 100];
quiver3(lx,ly,lz,[ex(1) ey(1) ez(1)],[ex(2) ey(2) ez(2)],[ex(3) ey(3) ez(3)], 'Color', 'red','LineWidth',3);
text(lx+ex,ly+ey,lz+ez,["xg","yg","zg"],"FontSize",14,"Color",'c');
hold on;
% now rotate the basis vectors by Rg and show
lxk = lx + origin_in_glitter_coords(1);
lyk = ly + origin_in_glitter_coords(2);
lzk = lz + origin_in_glitter_coords(3);
exk = (Rg * ex')';
eyk = (Rg * ey')';
ezk = (Rg * ez')';
quiver3(lxk,lyk,lzk,[exk(1) eyk(1) ezk(1)],[exk(2) eyk(2) ezk(2)],[exk(3) eyk(3) ezk(3)], 'Color', 'red','LineWidth',3);
text(lxk+exk,lyk+eyk,lzk+ezk,["xk","yk","zk"],"FontSize",14,"Color",'c');
% now rotate the basis vectors all the way to the camera position and
% show the new axes
cam_from_k_but_in_g = (Rg * location')';
lxc = lxk + cam_from_k_but_in_g(1); 
lyc = lyk + cam_from_k_but_in_g(2);
lzc = lzk + cam_from_k_but_in_g(3);
rotation_g2c = R';%equivalently: rotation_g2c = (Rc * Rg)';
exc = (rotation_g2c * ex')';
eyc = (rotation_g2c * ey')';
ezc = (rotation_g2c * ez')';
quiver3(lxc,lyc,lzc,[exc(1) eyc(1) ezc(1)],[exc(2) eyc(2) ezc(2)],[exc(3) eyc(3) ezc(3)], 'Color', 'red','LineWidth',3);
text(lxc+[exc(1) eyc(1) ezc(1)],lyc+[exc(2) eyc(2) ezc(2)],lzc+[exc(3) eyc(3) ezc(3)],["xc","yc","zc"],"FontSize",14,"Color",'c');
% show the computed camera position as a dot too
%scatter3([camPos(1)], [camPos(2)], [camPos(3)], [80], 'filled')
% finish up the figure
%}

trainSet = true;
hold on;
if trainSet 
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/sparkleResults').sparkleResults;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/checkerResults').checkerResults;
    sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/train_after_charMeasOptimization").results;
else
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/sparkleResults').sparkleResults;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerResults').checkerResults;
    %checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerResultsNoSkew').checkerResults;
    sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/test_after_charMeasOptimization").results;
    %threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults.mat").results;
    threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults4.mat").results;
end


% camera position
hold on;
title('camera position (tx,ty,tz)');
plot3(sparkleResults(:,1),sparkleResults(:,2),sparkleResults(:,3),'r*','markersize',10,'linewidth',2);
plot3(sparkleResultsOrig(:,1),sparkleResultsOrig(:,2),sparkleResultsOrig(:,3),'b*','markersize',10,'linewidth',2);
plot3(checkerResults(:,1),checkerResults(:,2),checkerResults(:,3),'gS','markersize',10,'linewidth',2);
if exist("threedCalResults", "var")
    plot3(threedCalResults(:,1),threedCalResults(:,2),threedCalResults(:,3),'mX','MarkerSize',10,'LineWidth',2);
end
xlabel('x (mm)');ylabel('y (mm)');zlabel('z (mm)');

trainSet = false;
hold on;
if trainSet 
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/sparkleResults').sparkleResults;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/checkerResults').checkerResults;
    sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/train_after_charMeasOptimization").results;
else
    sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/sparkleResults').sparkleResults;
    checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerResults').checkerResults;
    %checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerResultsNoSkew').checkerResults;
    sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/test_after_charMeasOptimization").results;
    %threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults.mat").results;
    threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults4.mat").results;
end

% camera position
hold on;
title('camera position (tx,ty,tz)');
plot3(sparkleResults(:,1),sparkleResults(:,2),sparkleResults(:,3),'r*','markersize',10,'linewidth',2);
plot3(sparkleResultsOrig(:,1),sparkleResultsOrig(:,2),sparkleResultsOrig(:,3),'b*','markersize',10,'linewidth',2);
plot3(checkerResults(:,1),checkerResults(:,2),checkerResults(:,3),'gS','markersize',10,'linewidth',2);
if exist("threedCalResults", "var")
    plot3(threedCalResults(:,1),threedCalResults(:,2),threedCalResults(:,3),'mX','MarkerSize',10,'LineWidth',2);
end
xlabel('x (mm)');ylabel('y (mm)');zlabel('z (mm)');


% third data set!
sparkleResultsOrig = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/sparkleResults').sparkleResults;
checkerResults = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/checkerResults').checkerResults;
sparkleResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/test_after_charMeasOptimization").results;
%threedCalResults = matfile("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/3dCalibrationResults.mat").results;

% camera position
hold on;
title('camera position (tx,ty,tz)');
plot3(sparkleResults(:,1),sparkleResults(:,2),sparkleResults(:,3),'r*','markersize',10,'linewidth',2);
plot3(sparkleResultsOrig(:,1),sparkleResultsOrig(:,2),sparkleResultsOrig(:,3),'b*','markersize',10,'linewidth',2);
plot3(checkerResults(:,1),checkerResults(:,2),checkerResults(:,3),'gS','markersize',10,'linewidth',2);
if exist("threedCalResults", "var")
    plot3(threedCalResults(:,1),threedCalResults(:,2),threedCalResults(:,3),'mX','MarkerSize',10,'LineWidth',2);
end
xlabel('x (mm)');ylabel('y (mm)');zlabel('z (mm)');


axis equal;
axis vis3d;
title('visualization of rotations and translations');
%legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');

