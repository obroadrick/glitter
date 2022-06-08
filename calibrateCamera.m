clear;
tic;
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
M = matfile([datap 'measurements.mat']).M;
tform = matfile([datap 'transform_06_08_2022.mat']).tform;
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
[params, imagesUsedParams, estimationErrors] = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize);
toc;
%% show a few aspects of the results
% plot reprojection errors
showReprojectionErrors(params);
% visualize 3d extrinsics 
figure;
showExtrinsics(params);
drawnow;
% plot an image and its detected and reprojected points for comparison
figure; 
index = 2;
imshow(imageFileNames{index}); 
hold on;
plot(imagePoints(:,1,index), imagePoints(:,2,index),'go');
plot(params.ReprojectedPoints(:,1,index),params.ReprojectedPoints(:,2,index),'r+');
legend('Detected Points','ReprojectedPoints');
hold off;
% display estimation errors
displayErrors(estimationErrors, params);
%% show montage of the images actually being used
imds = imageDatastore(imsp);
s = subset(imds, imagesUsed);
figure;
montage(s);

%% get coordinates for image with checkerboard on glitter plane
% use correct image: 'onglitterplane.jpg'
ogppath = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_06_2020/onglitterplane.JPG';
ogpim = imread(ogppath);
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(ogppath);
squareSizeInMM = M.CALIBRATION_SQUARE_SIZE;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);
imshow(ogppath);
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
% for the checkerboard point, get its camera coordinates
index_of_checker_origin = 1;%4;
point_in_checker_coords = [worldPoints(index_of_checker_origin,:) 0];
point_in_cam_coords = point_in_checker_coords*rotationMatrix + translationVector;

% now add the point in glitter coords to the camera in checkerboard coords
% to get the camera in glitter coords (coords both in MM and lined up axes,
% so the addition is fine; when these are thought of as vectors, they are in
% the same coordinate system)
[orientation, cam_in_checker_coords] = extrinsicsToCameraPose(rotationMatrix, ...
    translationVector);
camera_in_glitter_coords = point_in_glitter_coords + cam_in_checker_coords;
%correct the coordinate system by flipping the direction of the z-axis
camera_in_glitter_coords = camera_in_glitter_coords .* [1 1 -1];

%% save
save([datap 'camera_in_glitter_coords_' datestr(now, 'mm_dd_yyyy')], "camera_in_glitter_coords");

