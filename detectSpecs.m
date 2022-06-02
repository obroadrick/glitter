clear;
%% get max image
tic;
ims = [];
idxs = [];
m = [];
i = 0;
%colormap(gray);
%t = tiledlayout(2,1);
%t.Padding = 'compact';
%t.TileSpacing = 'compact';
for ix = 0:3:759
    i = i + 1;
    p = '/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures V (right - left)/';
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
    im = rgb2gray(im);
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
    %ax1 = nexttile(1); 
    %imagesc(ims(:,:,2));title(i);
    %drawnow;
    %ax2 = nexttile(2);
    %imagesc(idxs);title('indexes');
    %drawnow;
end
%imagesc(ims(:,:,2));colormap(gray);
m = ims(:,:,2);

%% filter image to keep only specs of glitter bright enough to be of interest
F = fspecial('Gaussian',[15 15],1) - fspecial('Gaussian',[15 15],7);
M = imfilter(m, F);
%imagesc(M);colormap(gray);

%% apply threshold to get binary map with glitter spec regions
thresh = 30;
Mt = M > thresh;
%imagesc(Mt);colormap(gray);
%% get a list of the region centroids
overlap_threshold = 8; % frame variation > thresh are two specs maybe
numPoints = 0;
C = [];
D = [];
R = regionprops(Mt,'Centroid','PixelIdxList','PixelList','Area');
numBad = 0;
for rx = 1:size(R,1)
    r = max(idxs(R(rx).PixelIdxList)) - min(idxs(R(rx).PixelIdxList));
    P = R(rx).Centroid;
    if r > overlap_threshold
        numBad = numBad + 1;
        D(numBad,:) = P;
        continue
    end
    numPoints = numPoints + 1;
    C(numPoints,:) = P;
end
%% salvage these 'bad' regions by finding 2 centroids instead
newCentroids = zeros(2*numBad,2);
numBad = 0;
L = bwlabel(Mt); % labels binary map where each region is its index
for rx = 1:size(R,1)
    r = max(idxs(R(rx).PixelIdxList)) - min(idxs(R(rx).PixelIdxList));
    P = R(rx).Centroid;
    if r <= overlap_threshold
        % ignore 'good' regions
        continue
    end
    disp(rx);
    numBad = numBad + 1;
    % use average index as a boundary
    avgidx = sum(idxs(R(rx).PixelIdxList)) / R(rx).Area;
    % two new regions: pixels from frames above avg and below
    reg = L==rx;
    R1 = regionprops(reg & idxs>avgidx, 'Centroid');
    R2 = regionprops(reg & idxs<avgidx, 'Centroid');
    % add new regions' centroids
    newCentroids(2*numBad-1,:) = R1(1).Centroid;
    newCentroids(2*numBad,:) = R2(1).Centroid;
end

% add these new good centroids to our list of centroids
C = [C; newCentroids];

%% save the centroids
time = datestr(now, 'yyyy_mm_dd');
filename = sprintf('data/centroids_%s.mat',time);
save(filename,'C');
toc;