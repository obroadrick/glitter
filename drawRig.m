function drawRig(M, lightings, specs)% draws the glitter rig
    % clear figure
    clf;
    % glitter square:
    gx = [0 M.GLIT_SIDE M.GLIT_SIDE 0]; 
    gy = [0 0 M.GLIT_SIDE M.GLIT_SIDE]; 
    gz = [0 0 0 0];
    gc = ['b'];
    patch(gx,gy,gz,gc);hold on;
    % monitor:
    mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
    my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
    mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
    mc = ['g'];
    patch(mx,my,mz,mc);
    % table: (made up coords, doesn't matter, mostly for fun)
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    patch(tx,ty,tz,tc);
    % show camera as a dot: (dots=cameras)
    scatter3([150],[450],[500],'filled');
    % draw all the passed lines
    for ix=1:size(lightings,1)
        % show line from light source to glitter spec:
        line([lightings(ix,1) specs(ix,1)],[lightings(ix,2) specs(ix,2)],[lightings(ix,3) specs(ix,3)]);
    end
    % set viewpoint:
    view([-110 -30]);
    camroll(-80);
    daspect([1 1 1]);
end