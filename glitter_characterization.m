

%% read in images of glitter
ims = [];
i = 1;
for ix = 444:4:480
    i = i + 1;
    im = imread(['glitter-images/calib' num2str(ix) '.0-Glitter.jpg']);
    im = double(rgb2gray(im))./255;
    ims(:,:,i) = im;
end
%% show an image of glitter
imagesc(ims(:,:,5));colormap(gray);

%% get coordinate system of glitter from fiducial markers
%todo (aruco?)

%% Find specs of glitter using max image of all images
m = max(ims,[],3);
imagesc(m);

%% filter image to keep only specs of glitter bright enough to be of interest
F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);

M = imfilter(m, F);
imagesc(M);colormap(gray);
disp(max(max(max(M,[],1),[],2),[],3));
disp(min(min(min(M,[],1),[],2),[],3));
%%
thresh = .2
imagesc(M > thresh);colormap(gray);

%% from star images lecture in comp photo:
% Let's find some features:
numPoints = 0;
C = [];
Mt = M>thresh;
imagesc(Mt);
R = regionprops(Mt);
for rx = 1:size(R,1)
    P = R(rx).Centroid;
    numPoints = numPoints + 1;
    C(numPoints,:) = P;
end
disp(size(C));

%% draw the centroids on top of the glitter specs image
imagesc(m);
for cx = 1:size(C,1)
    axis on;
    hold on;
    plot(C(cx,1),C(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
end

%% for each glitter spec


