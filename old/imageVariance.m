%% read in the images
path = '/Users/oliverbroadrick/Desktop/glitter-stuff/xenon_06_23_2022/'; 
files = dir(path);
ims = [];
numFiles = 1;
for ix=1:size(files,1)
    if files(ix).name(1) == '.'% if starts with a dot, ignore it
        continue
    end
    ims(numFiles,:,:) = rgb2gray(imread([files(ix).folder '/' files(ix).name]));
    disp(files(ix).name);
    numFiles = numFiles + 1;
end
%% get cell array instead? for montage argument
imsCells = {};
for ix=1:numFiles
    disp(ix);
    imsCells{ix} = ims(ix,:,:);
end
%% show montage!
montage(imsCells);
%% show some pixels' distributions
howmany = 100;
rng(122);
idxs = randi(5000,howmany,2);%rand ints in [1,first arg] of size second arg by third arg
dists = [];
for imix=1:numFiles
    for ix=1:howmany
        dists(ix,imix) = ims(imix,idxs(ix,1),idxs(ix,2));
    end
end
tiledlayout(int32(sqrt(howmany)+1),int32(sqrt(howmany)),...
    "TileSpacing","tight","Padding","tight");
for ix=1:howmany
    nexttile;
    plot(dists(ix,:));
    ylim([0 5]);
end

%% now find spec centroids and measure their intensity
%C = [];
%for ix=1:howmany
%    C(ix,:,:) = singleImageFindSpecs(reshape(ims(ix,:,:)));
%end
A = ims(1,:,:);
A = reshape(A,size(A,2),size(A,3));
figure;
imagesc(A);colormap(gray);

[C, Cmax] = singleImageFindSpecs(A);
%% show some centroids' brightnesses across the images
% get boolean map for the centroids of just the first image to use for all
% filter image to keep only specs of glitter bright enough to be of interest
im = ims(1,:,:);
F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
%F = fspecial('Gaussian',[40 40],7) - fspecial('Gaussian',[40 40],30);
imf = imfilter(im, F);
% apply threshold to get binary map with glitter spec regions
thresh = 30;
%thresh = 150;%for the image i sent in glitter channel
%thresh = 120;
Mt = imf > thresh;
howmany = 50;
centroidIdxs = randi(size(C,1),howmany,1);
idxs = C(centroidIdxs,:);%rand ints in [1,first arg] of size second arg by third arg
idxsmax = Cmax(centroidIdxs,:);
distsCentroids = [];
for imix=1:size(ims,1)
    % find the centroids for these specs
    for ix=1:howmany
        distsCentroids(ix,imix) = ims(imix,...
                                    int32(idxs(ix,2)),...
                                    int32(idxs(ix,1)));
        distsCentroidsMax(ix,imix) = ims(imix,...
                                    int32(idxsmax(ix,2)),...
                                    int32(idxsmax(ix,1)));
    end
end
figure;title('INTERPOLATE');
tiledlayout(int32(sqrt(howmany)+1),int32(sqrt(howmany)),...
    "TileSpacing","tight","Padding","tight");
ranges = [];
for ix=1:howmany
    nexttile;
    plot(distsCentroids(ix,:));
    ylim([0 255]);
    range = max(distsCentroids(ix,:)) - min(distsCentroids(ix,:));
    title(range);
    ranges(ix) = range;
end
figure;title('MAX');
tiledlayout(int32(sqrt(howmany)+1),int32(sqrt(howmany)),...
    "TileSpacing","tight","Padding","tight");
rangesmax = [];
for ix=1:howmany
    nexttile;
    plot(distsCentroidsMax(ix,:));
    ylim([0 255]);
    range = max(distsCentroidsMax(ix,:)) - min(distsCentroidsMax(ix,:));
    title(range);
    rangesmax(ix) = range;
end
rangesavg = double(sum(ranges)) / double(size(ranges,2));
rangesmaxavg = double(sum(rangesmax)) / double(size(rangesmax,2n));
disp('here are the average range in pixel intensity for');
disp('linearly interpolated spec intensities');
disp(rangesavg);
disp('max pixel intensities');
disp(rangesmaxavg);
%%
figure;
imagesc(ims(1,:,:));

