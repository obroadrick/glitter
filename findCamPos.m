% given camera calibration (camera parameters) and a new image with
% a checkerboard on the glitter plane, find the camera position 
% relative to the glitter plane 

% this involves finding a rotation and translation (Rg, tg) from canonical
% glitter coordinates to checkerboard coordinates and then another
% rotation and translation from checkerboard coordinates to camera 
% coordinates (Rc, tc)

function [t, R] = findCamPos(P, camParams, imPath, pin)
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
    indexOrigin = 1;
    indexXfar = 17;
    %plot(imagePoints(index,1), imagePoints(index,2),'go');
    %legend('Detected Points');
    %hold off;
    % get rotation and translation from checkerboard origin to camera
    worldPoints3d = [worldPoints zeros(size(worldPoints,1),1)];
    [~,tc] = estimateWorldCameraPose(imagePoints,worldPoints3d,params);
    [Rc, ~] = extrinsics(imagePoints,worldPoints,params);
    % get checkerboard point in canonical glitter coords
    % TODO make it so that we find these pin points using the marker 
    % detection code right here rather than getting them manually from addy
    tform = getTransform(P,pin);
    origin_in_glitter_coords2d = transformPointsForward(tform, [imagePoints(indexOrigin,1) imagePoints(indexOrigin,2)]);
    origin_in_glitter_coords = [origin_in_glitter_coords2d 0];
    tg = origin_in_glitter_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    xfar_in_glitter_coords2d = transformPointsForward(tform, [imagePoints(indexXfar,1) imagePoints(indexXfar,2)]);
    anglevec = xfar_in_glitter_coords2d - origin_in_glitter_coords2d;
    %thetafirsttry = asin(anglevec(2)/norm(anglevec));
    theta = atan2(anglevec(2), anglevec(1));
    theta = theta*-1;%need to unrotate this
    %figure;
    %line([origin_in_glitter_coords2d(1) xfar_in_glitter_coords2d(1)], [origin_in_glitter_coords2d(2) xfar_in_glitter_coords2d(2)]);
    %hold on; plot(anglevec(1),anglevec(2),'go');
    %disp(theta);
    %thetadeg = theta * 180 / pi;
    %disp(thetadeg);
    %draw to check
    %figure;
    %imagesc(imread(imPath));hold on;
    %plot(imagePoints([indexOrigin],1),imagePoints([indexOrigin],2),'go');
    %plot(imagePoints([indexXfar],1),imagePoints([indexXfar],2),'r+');
    %plot(pin(:,1),pin(:,2),'gx');
    %figure;
    %plot(xfar_in_glitter_coords2d(1), xfar_in_glitter_coords2d(2),'r+');
    %plot(origin_in_glitter_coords2d(1), origin_in_glitter_coords2d(2), 'go');

    % find rotation from glitter coords to checkerboard coords
    %TODO as below
    % find both origin and furthest away point on x axis of checkerboard
    % coordinate system but in the coordinates of the glitter... 
    % find angle between these two x axes...
    Rflipaxes = [1 0 0; 0 -1 0; 0 0 -1];
    Rrotate =[cos(theta)  -1*sin(theta)  0;...
              sin(theta)  cos(theta)     0;...
              0           0              1];
    Rg = Rflipaxes*Rrotate;
    t = (tg' + Rg*tc')';
    camPos = t;
    R = Rc * Rg;
    % R is rotation matrix from glitter to camera coordinates
end