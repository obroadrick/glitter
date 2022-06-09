% finds position of camera in the canonical glitter coordinate
% system using (1) matlab checkerboard camera calibration functions,
% (2) images of checkerboards, and (3) homography from image to 
% canonical coordinate system

% inputs: P, a matlab struct containing paths to the necessary images 
%            and homography
function campath = calibrateCamera(P)
    M = matfile(P.measurements).M;
    tform = matfile(P.tform).tform;
    %%
    imsp = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_06_2020/';
    images = imageSet(imsp);
    % could go through the images here and set isolate just the checkboard
    % find largest connected component of white-ish pixels
    % find bounding box around that connected component
    % set all pixels outside that bounding box to gray
    imageFileNames = images.Files();
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    I = readimage(images,1); 
    imageSize = [size(I, 1),size(I, 2)];
    [params, ~, ~] = estimateCameraParameters(imagePoints,worldPoints, ...
                                      'ImageSize',imageSize);
    %% show a few aspects of the results
    % plot reprojection errors
    showReprojectionErrors(params);
    % visualize 3d extrinsics 
    figure;
    showExtrinsics(params);
    drawnow;
    % plot an image and its detected and reprojected points for comparison
    %figure; 
    %index = 2;
    %imshow(imageFileNames{index}); 
    %hold on;
    %plot(imagePoints(:,1,index), imagePoints(:,2,index),'go');
    %plot(params.ReprojectedPoints(:,1,index),params.ReprojectedPoints(:,2,index),'r+');
    %legend('Detected Points','ReprojectedPoints');
    %hold off;
    % display estimation errors
    %displayErrors(estimationErrors, params);
    %% show montage of the images actually being used
    imds = imageDatastore(imsp);
    s = subset(imds, imagesUsed);
    figure;
    montage(s);
    
    %% get coordinates for image with checkerboard on glitter plane
    % use correct image: 'onglitterplane.jpg'
    [imagePoints, boardSize, ~] = detectCheckerboardPoints(P.onGlitterPlane);
    squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
    worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
    index = 1;
    hold on;
    plot(imagePoints(:,1,index), imagePoints(:,2,index),'go');
    legend('Detected Points');
    hold off;
    %%
    % for one of these checkerboard points, we apply the image -> glitter plane 
    % homography to get its location in canonical (glitter) coordinates
    % find index of the lowest-rightmost point (origin in checkerboard
    % coordinates)
    [~, index] = max(imagePoints(:,2) - imagePoints(:,1));
    point_in_glitter_coords = transformPointsForward(tform, [imagePoints(index,1) imagePoints(index,2)]);
    point_in_glitter_coords = [point_in_glitter_coords 0];
    
    % also get the point's coordinates in camera coordinates (AKA get
    % translations from camera to checkerboard points)
    [rotationMatrix, translationVector] = extrinsics(imagePoints,worldPoints,params);
    
    % now add the point in glitter coords to the camera in checkerboard coords
    % to get the camera in glitter coords (coords both in MM and lined up axes,
    % so the addition is fine; when these are thought of as vectors, they are in
    % the same coordinate system)
    [~, cam_in_checker_coords] = extrinsicsToCameraPose(rotationMatrix, ...
        translationVector);
    camera_in_glitter_coords = point_in_glitter_coords + cam_in_checker_coords;
    
    %correct the coordinate system by flipping the direction of the z-axis
    camera_in_glitter_coords = camera_in_glitter_coords .* [1 1 -1];
    
    %correct for thickness of calibration board
    camera_in_glitter_coords = camera_in_glitter_coords + [0 0 M.CALIBRATION_BOARD_THICKNESS];
    
    % save and return
    campath = [datap 'camera_in_glitter_coords_' datestr(now, 'mm_dd_yyyy')];
    save(campath, "camera_in_glitter_coords");
    end