% finds position of camera in the canonical glitter coordinate
% system using (1) matlab checkerboard camera calibration functions,
% (2) images of checkerboards, and (3) homography from image to 
% canonical coordinate system

% inputs: P, a matlab struct containing paths to the necessary images 
%            and homography
function campath = newCalibrateCamera(P)
    % read in the homography from image coords to canonical glitter coords
    M = matfile(P.measurements).M;
    tform = matfile(P.tform).tform;
    % read in the checkerboard images
    imsp = P.checkerboardIms;
    images = imageSet(imsp);
    imageFileNames = images.Files();
    % find checkerboard points
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    I = readimage(images,1); 
    imageSize = [size(I, 1),size(I, 2)];
    % calibrate camera using checkerboard points
    [params, ~, estimationErrors] = estimateCameraParameters(imagePoints,worldPoints, ...
                    'ImageSize',imageSize);
    %% show camera calibration results
    % plot reprojection errors
    showReprojectionErrors(params);
    % visualize 3d extrinsics 
    figure;
    showExtrinsics(params);
    drawnow;
    % print out estimation errors
    displayErrors(estimationErrors, params);
    % show montage of the images actually being used
    imds = imageDatastore(imsp);
    s = subset(imds, imagesUsed);
    figure;
    montage(s);
    %% get coordinates for image with checkerboard on glitter plane
    [imagePoints, boardSize, ~] = detectCheckerboardPoints(P.onGlitterPlane);
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    %% show just the image with the checkerboard on the plane
    imagesc(imread(P.onGlitterPlane));hold on;
    index = 1;%origin is always(?) first
    plot(imagePoints(index,1), imagePoints(index,2),'go');
    legend('Detected Points');
    hold off;
    % get rotation and translation from checkerboard origin to camera
    [rotationMatrix, translationVector] = extrinsics(imagePoints,worldPoints,params);
    worldPoints3d = [worldPoints zeros(size(worldPoints,1),1)];
    [worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints,worldPoints3d,params);
    % get checkerboard point in canonical glitter coords
    point_in_glitter_coords = [transformPointsForward(tform, [imagePoints(index,1) imagePoints(index,2)]) 0];
    tb = point_in_glitter_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    % rotation and translation from origin to camera coordinate system
    worldLocation = [worldLocation(1) worldLocation(2) -1*worldLocation(3)];
    t = tb + worldLocation;
    camera_in_glitter_coords = t;
    %% save and return
    campath = [P.data 'camera_in_glitter_coords_' datestr(now, 'mm_dd_yyyy')];
    save(campath, "camera_in_glitter_coords");
    % also save the camera calibration (rotation/translation from canonical
    % coordinates)
    end