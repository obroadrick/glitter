% given camera calibration (camera parameters) and a new image with
% a checkerboard on the glitter plane, find the camera position 
% relative to the glitter plane 

% this involves finding a rotation and translation (Rg, tg) from canonical
% glitter coordinates to checkerboard coordinates and then another 
% rotation and translation from checkerboard coordinates to camera 
% coordinates (Rc, tc)
function [t, R, terr, Rerr, Rvec] = findCamPos(P, camParams, camParamsErrors, imPath, pin)
    %% get coordinates for image with checkerboard on glitter plane
    M = matfile(P.measurements).M;
    params = camParams;%renaming
    [imagePoints, boardSize, ~] = detectCheckerboardPoints(imPath);
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    worldPoints3d = [worldPoints zeros(size(worldPoints,1),1)];

    % find points on the checkerboard in checkerboard/world coordinates
    % origin (point with x=y=0)
    indexOrigin = intersect(find(worldPoints(:,1)==0),find(worldPoints(:,2)==0));
    % furthest points along each axis x and y
    indexXfar = intersect(find(worldPoints(:,2)==0),find(worldPoints(:,1)==max(worldPoints(:,1))));
    indexYfar = intersect(find(worldPoints(:,1)==0),find(worldPoints(:,2)==max(worldPoints(:,2))));
    % get the points
    origin_in_world_coords = [worldPoints3d(indexOrigin,:)];
    xfar_in_world_coords = [worldPoints3d(indexXfar,:)];
    yfar_in_world_coords = [worldPoints3d(indexYfar,:)];
    
    
    %{ 
    % OLD WAY: get rotation and translation from checkerboard coords to 
    % camera coords
    [~,~] = estimateWorldCameraPose(imagePoints,worldPoints3d,params);
    [Rc, translationVector] = extrinsics(imagePoints,worldPoints,params);
    [~, location] = extrinsicsToCameraPose(Rc, translationVector);
    %}
    % new way, to base the extrinsics off the whole checkerboard set 
    % optimization and get error estimates accordingly
    onGlitPlaneIndex = 1;% requires naming the on-glitter-plane 
                         % checkerboard image alphabetically first
    Rvector = camParams.RotationVectors(onGlitPlaneIndex,:);
    Rc = rotationVectorToMatrix(Rvector);
    translationVector = camParams.TranslationVectors(onGlitPlaneIndex,:);
    [~, location] = extrinsicsToCameraPose(Rc, translationVector);
    % get/name some of the returns:
    terr = camParamsErrors.ExtrinsicsErrors.TranslationVectorsError(onGlitPlaneIndex,:);
    Rerr = camParamsErrors.ExtrinsicsErrors.RotationVectorsError(onGlitPlaneIndex,:);
    Rvec = Rvector;

    %%
    % get homography from image coordinates to glitter coordinates
    tform = getTransform(P,pin);

    % find the points (in world coordinates) directly back on the glitter
    % (perpendicularly projected on the glitter plane) (we can use these 
    % points to find the rotation in the xy-plane from glitter coordinates 
    % to world/checkerboard coordinates)
    origin_in_world_coords = origin_in_world_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    xfar_in_world_coords = xfar_in_world_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    yfar_in_world_coords = yfar_in_world_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];

    % now we can find the corresponding image coordinates for those points
    % since we know the rotation and intrinsic matrix K from this
    % checkerboard calibration
    rotationMatrix = Rc;
    origin_in_image_coords = worldToImage(params.Intrinsics,rotationMatrix,translationVector,origin_in_world_coords);
    xfar_in_image_coords = worldToImage(params.Intrinsics,rotationMatrix,translationVector,xfar_in_world_coords);
    yfar_in_image_coords = worldToImage(params.Intrinsics,rotationMatrix,translationVector,yfar_in_world_coords);
    
    % use our homograpy from image coordinates to glitter plane coordinates
    % to get the checkerboard points (which have projected onto the glitter
    % plane) in glitter coordinates.
    origin_in_glitter_coords2d = transformPointsForward(tform, origin_in_image_coords);
    xfar_in_glitter_coords2d = transformPointsForward(tform, xfar_in_image_coords);
    yfar_in_glitter_coords2d = transformPointsForward(tform, yfar_in_image_coords);

    % get the checkerboard points in 3d glitter coordinates of the original
    % checkerboard points (rather than the points perpendicularly projected
    % onto the glitter plane)
    origin_in_glitter_coords = [origin_in_glitter_coords2d M.CALIBRATION_BOARD_THICKNESS];
    xfar_in_glitter_coords = [xfar_in_glitter_coords2d M.CALIBRATION_BOARD_THICKNESS];
    yfar_in_glitter_coords = [yfar_in_glitter_coords2d M.CALIBRATION_BOARD_THICKNESS];

    % rename the checkerboard origin in glitter coordinates
    tg = origin_in_glitter_coords;

    % find the vector from origin to xfar on the checkerboard to be used to
    % find the rotation from glitter coords to checkerboard coords
    anglevec = xfar_in_glitter_coords2d - origin_in_glitter_coords2d;
    
    % find the angle of the 2d rotation of the xy-plane from glitter coords
    % to checkerboard coords
    theta = -1*atan2(anglevec(2), anglevec(1));

    % rotation matrix from glitter coords to world/checkerboard coords that
    % puts the x axis in the correct position (by carrying out the 2d
    % rotation of the xy-plane by theta)
    Rrotate =[cos(theta)     -1*sin(theta)     0;...
              sin(theta)     cos(theta)        0;...
              0              0                 1];

    % the rotation from glitter coods to checkerboard coords includes
    % flipping the y and z axes (in addition to the 2d rotation)
    Rflipaxes = [1 0 0; 0 -1 0; 0 0 -1];

    % compute the full 3d rotation matrix from glitter coords to 
    % checkerboard coords
    Rg = Rflipaxes*Rrotate;

    % compute the position of the camera in glitter coordinates
    t = (tg' + Rg*location')';

    % compute the overall rotation from glitter coords to camera coords
    R = Rc' * Rg;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % draw the checkerboard points used in glitter coords
    figure;
    line([origin_in_glitter_coords2d(1) xfar_in_glitter_coords2d(1)],...
        [origin_in_glitter_coords2d(2) xfar_in_glitter_coords2d(2)]);
    line([origin_in_glitter_coords2d(1) yfar_in_glitter_coords2d(1)],...
        [origin_in_glitter_coords2d(2) yfar_in_glitter_coords2d(2)]);
    hold on; 
    plot(anglevec(1),anglevec(2),'go');
    % draw the checkboard points used on the image (in image coords)
    figure;
    imagesc(imread(imPath));
    hold on;
    plot(imagePoints([indexOrigin],1),imagePoints([indexOrigin],2),'go');
    plot(imagePoints([indexXfar],1),imagePoints([indexXfar],2),'r+');
    plot(imagePoints([indexYfar],1),imagePoints([indexYfar],2),'yx');
    plot(pin(:,1),pin(:,2),'gx');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % draw the scene with camera and frustrum
    figure;
    camPos = t;
    T = camPos;
    pose = rigid3d(R,T);
    hold on;
    camObj = plotCamera('AbsolutePose',pose,'Opacity',0,'Size',35);
    T = T';
    frustumImagePoints = [0 0; 0 M.YRES; M.XRES M.YRES; M.XRES 0];
    % since p = KR(P-T) for world point P to image points p,
    % we get P = T+(KR)^-1(p)
    K = camParams.Intrinsics.IntrinsicMatrix';
    frustumWorldPoints = [];
    for ix=1:size(frustumImagePoints,1)
        frustumWorldPoints(ix,:) = T + inv(K*R) * (800 .* [frustumImagePoints(ix,:)';1]);
    end
    for ix=1:size(frustumWorldPoints,1)
        plot3([T(1) frustumWorldPoints(ix,1)],...
              [T(2) frustumWorldPoints(ix,2)],...
              [T(3) frustumWorldPoints(ix,3)],...
              'Color', 'cyan');
    end
    hold on;
    plot3(T(1),T(2),T(3),'cx'); 
    title('checkpoint 555');
    axis vis3d;
    axis equal;
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
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    axis vis3d;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    % visualize (in glitter coordinates) each of the three coordinate
    % systems
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
    scatter3([camPos(1)], [camPos(2)], [camPos(3)], [80], 'filled')
    % finish up the figure
    axis equal;
    axis vis3d;
    title('visualization of rotations and translations');
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end