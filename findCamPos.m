% given camera calibration (camera parameters) and a new image with
% a checkerboard on the glitter plane, find the camera position 
% relative to the glitter plane 

% this involves finding a rotation and translation (Rg, tg) from canonical
% glitter coordinates to checkerboard coordinates and then another
% rotation and translation from checkerboard coordinates to camera 
% coordinates (Rc, tc)

%function [t, R] = findCamPos(P, camParams, imPath, pin)
    % P is paths struct
    % cameraParams is the matlab camera parameters object
    % imPath is the path to the image with checkerboard on 
    %        the glitter plane (not necessarily square to it though!)
    %% get coordinates for image with checkerboard on glitter plane
    M = matfile(P.measurements).M;
    params = camParams;%renaming
    [imagePoints, boardSize, ~] = detectCheckerboardPoints(imPath);
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    % show image with the checkerboard on the plane
    %imagesc(imread(imPath));hold on;
    %index = 1;%origin is always(?) first
    %indexOrigin = 1;
    %indexXfar = 17;
    % instead of hard coding the index of each location on the board, we
    % will instead just find the origin and furthest x coordinate
    % automatically.
    % find point with both x and y 0 (the origin)
    indexOrigin = intersect(find(worldPoints(:,1)==0),find(worldPoints(:,2)==0));
    % find point with x as high as possible and y 0 (farthest point along
    % x axis)
    indexXfar = intersect(find(worldPoints(:,2)==0),find(worldPoints(:,1)==max(worldPoints(:,1))));
    indexYfar = intersect(find(worldPoints(:,1)==0),find(worldPoints(:,2)==max(worldPoints(:,2))));

    %{
    % sometimes these 'world' axes will have x and y axes not pointing in a
    % consistent way... up to rotations there are two possible
    % orientations, one in which the z axis is pointing out of the board
    % according to Right hand rule and the other where z axis is into the
    % board. we use the image coordinates of Xfar and Yfar to get a cross
    % product whose sign tells us the answer to that question. (which
    % handed-ness coordinate system we have). we need to know this because
    % we need to know how to reconstruct a rotation matrix that will get us
    % the coordinate system found here which involves rotating 180 around
    % the z axis potentially or potentially not depending on this question.
    cp = cross([imagePoints(indexXfar,:) 0]-[imagePoints(indexOrigin,:) 0],...
        [imagePoints(indexYfar,:) 0]-[imagePoints(indexOrigin,:) 0]);
    if cp < 0
        % this is the case where the x and y axes are just a simple
        % rotation away and no handedness has changed
        Rflipaxes = [1 0 0; 0 1 0; 0 0 1];
        disp('here');
    else
        % this is the case where the axes are only the same handedness if
        % we flip the z axis
        Rflipaxes = [1 0 0; 0 1 0; 0 0 -1];
        disp('no here');
    end
    %}

    %plot(imagePoints(index,1), imagePoints(index,2),'go');
    %legend('Detected Points');
    %hold off;
    % get rotation and translation from checkerboard origin to camera
    worldPoints3d = [worldPoints zeros(size(worldPoints,1),1)];
    [Rc,tcam] = estimateWorldCameraPose(imagePoints,worldPoints3d,params);
    [~, translationVector] = extrinsics(imagePoints,worldPoints,params);% bad, bad function, as far as my use goes apparently
    % get checkerboard point in canonical glitter coords
    % TODO make it so that we find these pin points using the marker 
    % detection code right here rather than getting them manually from addy
    tform = getTransform(P,pin);
    %origin_in_image_coords2d = [imagePoints(indexOrigin,1) imagePoints(indexOrigin,2)];

    % now get checker points in world coords 
    origin_in_world_coords = [worldPoints3d(indexOrigin,:)];
    xfar_in_world_coords = [worldPoints3d(indexXfar,:)];
    yfar_in_world_coords = [worldPoints3d(indexYfar,:)];

    % now we can find these points (in world coordinates) directly back
    % on the glitter plane (perpendicularly projected on the glitter plane)
    % which is useful since we can use these points to find the rotation
    % in the xy-plane from glitter coordinates to world/checker coordinates
    origin_in_world_coords = origin_in_world_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    xfar_in_world_coords = xfar_in_world_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    yfar_in_world_coords = yfar_in_world_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];

    % now we can find the corresponding image coordinates for those points
    % since we know the rotation and intrinsic matrix K from this
    % checkerboard calibration
    rotationMatrix = Rc';
    origin_in_image_coords = worldToImage(params.Intrinsics,rotationMatrix,translationVector,origin_in_world_coords);
    xfar_in_image_coords = worldToImage(params.Intrinsics,rotationMatrix,translationVector,xfar_in_world_coords);
    yfar_in_image_coords = worldToImage(params.Intrinsics,rotationMatrix,translationVector,yfar_in_world_coords);
    

    % finally we can use our homograpy from image coordinates to glitter
    % plane coordinates

    %%%%%
    % what we were doing before:
    origin_in_glitter_coords2d = transformPointsForward(tform, origin_in_image_coords);
    %origin_in_glitter_coords = [origin_in_glitter_coords2d 0] + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    origin_in_glitter_coords = [origin_in_glitter_coords2d 0];% + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    tg = origin_in_glitter_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    %xfar_in_glitter_coords2d = transformPointsForward(tform, [imagePoints(indexXfar,1) imagePoints(indexXfar,2)]);
    xfar_in_glitter_coords2d = transformPointsForward(tform, xfar_in_image_coords);
    %xfar_in_glitter_coords = [xfar_in_glitter_coords2d 0] + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    xfar_in_glitter_coords = [xfar_in_glitter_coords2d 0];% + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    %yfar_in_glitter_coords2d = transformPointsForward(tform, [imagePoints(indexYfar,1) imagePoints(indexYfar,2)]);
    yfar_in_glitter_coords2d = transformPointsForward(tform, yfar_in_image_coords);
    %yfar_in_glitter_coords = [yfar_in_glitter_coords2d 0] + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    yfar_in_glitter_coords = [yfar_in_glitter_coords2d 0];% + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    anglevec = xfar_in_glitter_coords2d - origin_in_glitter_coords2d;
    
    %thetafirsttry = asin(anglevec(2)/norm(anglevec));
    theta = atan2(anglevec(2), anglevec(1));
    theta = theta*-1;%need to unrotate this
    figure;
    line([origin_in_glitter_coords2d(1) xfar_in_glitter_coords2d(1)], [origin_in_glitter_coords2d(2) xfar_in_glitter_coords2d(2)]);
    hold on; plot(anglevec(1),anglevec(2),'go');
    %disp(theta);
    thetadeg = theta * 180 / pi;
    %disp(thetadeg);
    %draw to check
    figure;
    imagesc(imread(imPath));hold on;
    plot(imagePoints([indexOrigin],1),imagePoints([indexOrigin],2),'go');
    plot(imagePoints([indexXfar],1),imagePoints([indexXfar],2),'r+');
    plot(pin(:,1),pin(:,2),'gx');
    figure;
    plot(xfar_in_glitter_coords2d(1), xfar_in_glitter_coords2d(2),'r+');
    plot(origin_in_glitter_coords2d(1), origin_in_glitter_coords2d(2), 'go');

    % find rotation from glitter coords to checkerboard coords
    %TODO as below
    % find both origin and furthest away point on x axis of checkerboard
    % coordinate system but in the coordinates of the glitter... 
    % find angle between these two x axes...
    Rflipaxes = [1 0 0; 0 -1 0; 0 0 -1];

    % rotation matrix from glitter origin to world/checkerboard origin
    Rrotate =[cos(theta)     -1*sin(theta)     0;...
              sin(theta)     cos(theta)        0;...
              0              0                 1];
    Rg = Rflipaxes*Rrotate;
    t = (tg' + Rg'*tcam')';
    %t = (Rflipaxes* t')';%temp
    camPos = t;
    R = Rc * Rg;
    
    %% draw the scene with camera and its frustrum
    M = matfile(P.measurements).M;
    figure;
    T = camPos;
    pose = rigid3d(R,T);
    hold on;
    camObj = plotCamera('AbsolutePose',pose,'Opacity',0,'Size',35);
    % draw frustum
    T = T';
    frustumImagePoints = [0 0; 0 M.YRES; M.XRES M.YRES; M.XRES 0];
    % since p = KR(P-T) for world point P to image points p,
    % we get that P = T+(KR)^-1(p)
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
    plot3(T(1),T(2),T(3),'cx'); title('checkpoint 555');
    hold on;
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

    %visAxesRT(R,T);
    %%
    axis vis3d;
    % R is rotation matrix from glitter to camera coordinates
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % new visualization of the rotations and translations happening in
    % glitter coordinates
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
    %origin_in_glitter_coords xfar_in_glitter_coords, yfar_in_glitter_coords
    
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
    %disp(lxk+exk);
    %disp(size(lxk));
    %disp(size(exk));
    text(lxk+exk,lyk+eyk,lzk+ezk,["xk","yk","zk"],"FontSize",14,"Color",'c');

    % now rotate the basis vectors all the way to the camera position and
    % show the new axes
    cam_from_k_but_in_g = (Rg' * tcam')';
    lxc = lxk + cam_from_k_but_in_g(1);
    lyc = lyk + cam_from_k_but_in_g(2);
    lzc = lzk + cam_from_k_but_in_g(3);
    rotation_g2c = (Rc * Rg)';%Rc * Rg;%R';% equivalently: rotation_g2c = (Rc * Rg)';
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
    title('visualization of rotations and translations');
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%end