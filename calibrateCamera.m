clear;
tic;
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
M = matfile([datap 'measurements.mat']).M;
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
% find correct image 'onglitterplane.jpg'
% find lowest left point
% use homography to get its position in glitter plane coordinates
