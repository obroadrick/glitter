

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
    disp(P);
end
disp(size(C));

%% draw the centroids on top of the glitter specs max image
imagesc(m);
for cx = 1:size(C,1)
    axis on;
    hold on;
    plot(C(cx,1),C(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
end

%% for each glitter spec
% now let's build a distribution for each glitter spec of brightness for
% the various lighting positions
% if we assume that a glitter spec's brighness can be found simply by
% checking the brightness of its centroid, then this is as simple
% as building a 3d matrix that is a stack of the images over time
% and then looking at the line for each centroid

% the stack of images is ims

% for each centroid
for cx = 100:100%size(C,1)
    dist = reshape(ims(C(cx,2),C(cx,1),:),11,1);
    plot(dist, '+', 'markersize', 8);
end




