%% read in images of glitter
% since we can't fit all the images into memory at once
% we just maintain a 'max' image
ims = [];
idxs = [];
m = [];
i = 0;
colormap(gray);
t = tiledlayout(2,1);
t.Padding = 'compact';
t.TileSpacing = 'compact';
for ix = 0:3:765
    i = i + 1;
    p = '/Users/oliverbroadrick/Desktop/glitter-stuff/Calibrations5-25/';
    path = [p '*calib' num2str(ix) '.0-Glitter.jpg'];
    files = dir(path);
    if length(files) < 1
        disp(['no file found at:' path]);
    end
    if length(files) > 1
        disp(['more than one file at:' path]);
    end
    %disp([files(1).folder '/' files(1).name]);
    disp(ix);
    im = imread([files(1).folder '/' files(1).name]);
    %now we don't actually convert to float in order to save size
    im = rgb2gray(im);%double(rgb2gray(im))./255;
    ims(:,:,1) = im;
    if i == 1
        % for the first image, make it the current max
        ims(:,:,2) = im;
        idxs = im;
        idxs(:,:) = 1;
    end
    % max image, fake indexes 
    [ims(:,:,2), tempidxs] = max(ims,[],3);
    % real indexes (1 -> from new image, 2 -> leave as before)
    idxs(tempidxs == 1) = i;

    % show
    ax1 = nexttile(1); 
    imagesc(ims(:,:,2));title(i);
    drawnow;
    ax2 = nexttile(2);
    imagesc(idxs);title('indexes');
    drawnow;
    
end
%% show the max image of glitter
imagesc(ims(:,:,2));colormap(gray);
m = ims(:,:,2);
%%
g = uint8(ims(:,:,2));
imwrite(cat(3, g, g, g), 'imgs/may26/max.jpg');
%%
m = ims(:,:,2);

%% get coordinate system of glitter from fiducial markers
%todo (aruco?)

%% Find specs of glitter using max image of all images
%m = max(ims,[],3);
%imagesc(m);colormap(gray);

%% filter image to keep only specs of glitter bright enough to be of interest
F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);

M = imfilter(m, F);
imagesc(M);colormap(gray);
%%
thresh = 30;
imagesc(M > thresh);colormap(gray);

%% get a list of the spec centroids
overlap_threshold = 8; % diff in index >= thresh are two specs maybe
numPoints = 0;
C = [];
D = [];
Mt = M>thresh;
%imagesc(Mt);
R = regionprops(Mt,'Centroid','PixelIdxList');
numBad = 0;
for rx = 1:size(R,1)
    r = max(idxs(R(rx).PixelIdxList)) - min(idxs(R(rx).PixelIdxList));
    P = R(rx).Centroid;
    numPoints = numPoints + 1;
    if r > overlap_threshold
        numBad = numBad + 1;
        D(numBad,:) = P;
    end
    C(numPoints,:) = P;
end
%% draw the detected bad centroids on top of the glitter specs max image
imagesc(m);colormap(gray);
for cx = 1:size(D,1)
    axis on;
    hold on;
    plot(D(cx,1),D(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
end
%% salvage these 'bad' regions by finding 2 centroids instead
newCentroids = []
for rx = 1:size(R,1)
    r = max(idxs(R(rx).PixelIdxList)) - min(idxs(R(rx).PixelIdxList));
    P = R(rx).Centroid;
    numPoints = numPoints + 1;
    if r < overlap_threshold
        % ignore 'good' regions
        continue
    end
    numBad = numBad + 1;

    % add the two new centroids
    newCentroids(2*numBad-1,:) = P;
    newCentroids(2*numBad,:) = P;
    
end
%% draw the centroids on top of the glitter specs max image
imagesc(m);
for cx = 15000:17000%size(C,1)
    axis on;
    hold on;
    plot(C(cx,1),C(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
end

%% brightness distributions for some random specs' centroids
% (for the various lighting positions)
num = 16;
rng(3141592);
cxs = randi(size(C,1),num,1);
numims = 256;
dists = zeros(num,numims);
i = 0;
colormap(gray);
% read thru the images
for imx = 0:3:765
    i = i + 1;
    p = '/Users/oliverbroadrick/Desktop/glitter-stuff/Calibrations5-25/';
    path = [p '*calib' num2str(imx) '.0-Glitter.jpg'];
    files = dir(path);
    if length(files) < 1
        disp(['no file found at:' path]);
    end
    if length(files) > 1
        disp(['more than one file at:' path]);
    end
    disp(imx);
    im = rgb2gray(imread([files(1).folder '/' files(1).name]));
    % update each of the dists with this im's data
    for ix=1:size(cxs,1)
        cx = cxs(ix);
        d1 = int32(C(cx,2));
        d2 = int32(C(cx,1));
        dists(ix, i) = im(d1,d2);
    end
end
%% plot the dists
disp('now plotting')
t = tiledlayout(4,4);
num = 16;
t.Padding = 'compact';
t.TileSpacing = 'compact';
x = [1:numims]';
%cxs=randi(size(C,1),num,1);
for ix=1:size(cxs,1)%start:start+num-1%size(C,1)
    nexttile
    %disp(cx);
    dist = reshape(dists(ix,:), numims, 1);%reshape(ims(int32(C(cx,2)),int32(C(cx,1)),:),11,1);
    %g = fit(x, dist, 'gauss1');
    plot(dist); %hold on;
    title(num2str(ix));
    %plot(g);
end

%% let's look at the specific sampled specs' centroids now
% draw the centroids on top of the glitter specs max image
imagesc(m);colormap(gray);
overlap_threshold = 10;
for ix=1:size(cxs,1)
    cx = cxs(ix);
    hold on;
    %plot(C(cx,1),C(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
    text(C(cx,1),C(cx,2), cellstr(num2str(ix)), 'FontSize', 12, 'Color', [1 0 0]);
end

%% make a tethered set of plots:
colormap(gray);
t = tiledlayout(2,2);
t.Padding = 'compact';
t.TileSpacing = 'compact';
% max image
ax1 = nexttile(1); 
imagesc(ims(:,:,2));title('max image');
% indexes
ax2 = nexttile(2);
imagesc(idxs);title('indexes');
% filtered and thresholded binary image
ax3 = nexttile(3);
imagesc(M > thresh);title('binary image after filter/threshold');
% found centroids
ax4 = nexttile(4);
imagesc(m);
for cx = 15000:17000%size(C,1)
    axis on;
    hold on;
    plot(C(cx,1),C(cx,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
end
title('spec locations found (region centroids after max, filter, threshold)');
% link axes
linkaxes([ax1 ax2 ax3 ax4]);

%% find regions with indexes over a high range (overlapping specs maybe)
numPoints = 0;
C = [];
Mt = M>thresh;
imagesc(Mt);
R = regionprops(Mt,'Centroid','PixelIdxList');
ranges = zeros(size(R,1),1);
for rx = 1:size(R,1)
    ranges(rx) = max(idxs(R(rx).PixelIdxList)) - min(idxs(R(rx).PixelIdxList));
    P = R(rx).Centroid;
    numPoints = numPoints + 1;
    C(numPoints,:) = P;
end
histogram(ranges);title('Histogram of Index Variation within Regions');
%% so, we use a threshold to detect overlapping specs
overlap_threshold = 10;

%%
disp('helloworld');
