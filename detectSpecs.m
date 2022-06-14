% finds centroids of specs of glitter in both (1) image coordinate
% system and (2) canonical (glitter) coordinate system.
% uses images from a full sweep and a known homography
% from image coordinates to canonical (glitter) coordinates

% inputs: P, a matlab struct containing paths to relevant images/data
function [imageCentroidsPath, canonicalCentroidsPath] = detectSpecs(P)
    %% get max image
    ims = [];
    idxs = [];
    i = 0;
    %colormap(gray);
    %t = tiledlayout(2,1);
    %t.Padding = 'compact';
    %t.TileSpacing = 'compact';
    %TODO add code to find the highest index of sweepimage for each
    %     direction so that we don't manually enter it like animals
    %...... 
    maxIndexLeftRight = 759;
    for ix = 0:3:maxIndexLeftRight
        i = i + 1;
        path = [P.leftRightSweep '*calib' num2str(ix) '.0-Glitter.jpg'];
        files = dir(path);
        if length(files) < 1
            disp(['no file found at:' path]);
        end
        if length(files) > 1
            disp(['more than one file at:' path]);
        end
        %disp([files(1).folder '/' files(1).name]);
        %disp(ix);
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
        Pt = R(rx).Centroid;
        if r > overlap_threshold
            numBad = numBad + 1;
            D(numBad,:) = Pt;
            continue
        end
        numPoints = numPoints + 1;
        C(numPoints,:) = Pt;
    end
    %% salvage these 'bad' regions by finding 2 centroids instead
    newCentroids = zeros(2*numBad,2);
    numBad = 0;
    L = bwlabel(Mt); % labels binary map where each region is its index
    for rx = 1:size(R,1)
        r = max(idxs(R(rx).PixelIdxList)) - min(idxs(R(rx).PixelIdxList));
        if r <= overlap_threshold
            % ignore 'good' regions
            continue
        end
        %disp(rx);
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

    % get the centroids in canonical glitter coordinates too
    % read in the homography that maps image coords to canonical coords
    tform = matfile(P.tform).tform;
    
    % transform points
    cxs = [1:size(C,1)];
    out = transformPointsForward(tform, [C(cxs,1) C(cxs,2)]);
    % get specs' 3D coordinates (add a zero for z axis)
    C_canonical = [out(:,1) out(:,2) zeros(size(out,1),1)];
    
    %% return and save them
    imageCentroids = C;
    canonicalCentroids = C_canonical;
    imageCentroidsPath = [P.data 'image_centroids_' datestr(now, 'mm_dd_yyyy')];
    canonicalCentroidsPath = [P.data 'canonical_centroids_' datestr(now, 'mm_dd_yyyy')];
    save(imageCentroidsPath, "imageCentroids");
    save(canonicalCentroidsPath, "canonicalCentroids");
end