% visualize rotation and translation of glitter axes to camera axes

function x = visAxesRT(R,T)
    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    M = matfile(P.measurements).M;
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
    hold on;
    
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

    % now rotate the basis vectors all the way to the camera position and
    % show the new axes
    rotation_g2c = R';
    lxc = [T(1) T(1) T(1)];
    lyc = [T(2) T(2) T(2)];
    lzc = [T(3) T(3) T(3)];
    exc = (rotation_g2c * ex')';
    eyc = (rotation_g2c * ey')';
    ezc = (rotation_g2c * ez')';
    quiver3(lxc,lyc,lzc,[exc(1) eyc(1) ezc(1)],[exc(2) eyc(2) ezc(2)],[exc(3) eyc(3) ezc(3)], 'Color', 'red','LineWidth',3);
    %disp(size(lxk));
    %disp(size(exk));
    text(lxc+[exc(1) eyc(1) ezc(1)],lyc+[exc(2) eyc(2) ezc(2)],lzc+[exc(3) eyc(3) ezc(3)],["xc","yc","zc"],"FontSize",14,"Color",'c');

    % finish up the figure
    axis equal;
    axis vis3d;
    title('visualization of R and T tranforming the axes');
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    
    x = 0;
end