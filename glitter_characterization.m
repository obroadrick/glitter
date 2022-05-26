%% read in images of glitter
ims = [];
i = 1;
for ix = 444:4:480
    i = i + 1;
    im = imread(['/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/glitter-images/calib' num2str(ix) '.0-Glitter.jpg']);
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
%%
thresh = .15
imagesc(M > thresh);colormap(gray);

%% get a list of the points
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

%% draw the centroids on top of the glitter specs max image
imagesc(m);
for cx = 1:1000%size(C,1)
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

% now we look at the brightness dists for some random specs' centroids
t = tiledlayout(4,4);
num = 16;
t.Padding = 'compact';
t.TileSpacing = 'compact';
cxs=randi(size(C,1),num,1);
for ix=1:size(cxs,1)%start:start+num-1%size(C,1)
    nexttile
    cx = cxs(ix);
    %disp(cx);
    dist = reshape(ims(int32(C(cx,2)),int32(C(cx,1)),:),11,1);
    g = fit(x, dist, 'gauss1');
    plot(dist); hold on;
    plot(g);
end


%% Now it's natural to wonder whether we can do better than to look at
% just the pixel which contains the centroid of each glitter spec
% let's begin by seeing whether the average of the centroid and its 
% 4 cardinally-adjacent pixels gives a smoother gaussian-ish curve

num = 100;
t = tiledlayout(ceil(sqrt(num)), ceil(sqrt(num)));
t.Padding = 'compact';
t.TileSpacing = 'compact';
cxs=randi(size(C,1),num,1);
ssds = [];
for ix=1:size(cxs,1)
    nexttile
    cx = cxs(ix);
    %disp(cx);
    dist = [];
    % spec brightness will now be centroid and 4 adjacent pxs
    d1 = int32(C(cx,2));
    d2 = int32(C(cx,1));
    for m=1:size(ims, 3)
        dist(m) = 0;
        dist(m) = dist(m) + ims(d1,d2,m);
        %dist(m) = dist(m) + ims(d1+1,d2,m);
        %dist(m) = dist(m) + ims(d1-1,d2,m);
        %dist(m) = dist(m) + ims(d1,d2+1,m);
        %dist(m) = dist(m) + ims(d1,d2-1,m);
        %dist(m) = dist(m) + ims(d1+1,d2+1,m);
        %dist(m) = dist(m) + ims(d1-1,d2+1,m);
        %dist(m) = dist(m) + ims(d1+1,d2-1,m);
        %dist(m) = dist(m) + ims(d1-1,d2-1,m);
        %dist(m) = dist(m) / 9;
    end
    dist = reshape(dist, 11, 1);
    g = fit(x, dist, 'gauss1');
    %plot(dist); hold on;
    %plot(g); hold on;
    ssds(ix) = sum((g(x)-dist).^2);
end
avgerr = sum(ssds) / size(ssds,2);
disp(avgerr);
