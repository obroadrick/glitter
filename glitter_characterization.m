

%% read in image of glitter
im = imread('glitter-images/calib444.0-Glitter.jpg');
imagesc(im);

%% get coordinate system of glitter from fiducial markers
%todo (aruco?)

%% filter image to keep only specs of glitter bright enough to be of interest
F = fspecial('Gaussian',[5 5],2) - fspecial('Gaussian',[5 5],3);

imf = imfilter(im, F);
imagesc(imf);

%% from star images lecture in comp photo:
M2 = imf;
% Let's find some features:
numPoints = 0;
Q = [];

starImage = M2(:,:) > 0.1;
R = regionprops(starImage);
for rx = 1:size(R,1);
    P = R(rx).Centroid;
    P(3) = mx;                              % put in the frame number
    numPoints = numPoints + 1;
    Q(numPoints,:) = P;
end
disp(mx);

%% for each glitter spec