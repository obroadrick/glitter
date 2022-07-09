% find lens distortion and camera translation from glitter to camera 
% using a known light source, a picture of sparkling glitter, and a 
% known glitter characterization (spec positions and surface normals)
clear;
rng(314159);
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
M = matfile(P.measurements).M;

% get known characterization
knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
allSpecNormals = matfile(P.specNormals).specNormals;

% read in image
impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
%impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/xenon_06_23_2022/2022-06-23T14,15,20Single-Glitter.JPG';
im = rgb2gray(imread(impath));

% get lighting position in canonical coords form lighting position in
% monitor pixel coords
monitorCoords = [1127 574];
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
lightPos = [x y M.GLIT_TO_MON_PLANES];

%% find spec centroids in image
%pin = [1217.34838867 5145.87841797; 1005.55084  295.4278; 6501.5874  490.0575; 6501.952 5363.594];
%pin = [1642.2677 5380.783; 1337.9928 733.52966; 6572.239 726.0792; 6226.173 5270.477];
%rallPts = matfile([P.data '16pts_june23.mat']).arr;
%pin = allPts(1,:);
%pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
%piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
%pin = [pinx' piny'];
pinorig = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y 
figure;
testimpath = "/Users/oliverbroadrick/Downloads/DSC_1817.JPG";
%imagesc(rgb2gray(imread(testimpath)));colormap(gray);hold on;
%plot(pinorig(:,1),pinorig(:,2),'rx','MarkerSize',15);
tform = getTransform(P, pinorig);
[imageCentroidsOrig,~] = singleImageFindSpecs(im);


%% for given radial distortion coefficients (k1,k2) compute 
%     1. how many inliers there are, and
%     2. how far the inlier image specs are from their matching 
%        world specs 
% this will give an error measure for various candidate distortion
% coefficients and so will let us make a little optimization for them

%{ 
% throw back to when I chose my distortion coefficient search space in a
%   really bad way!
r1 = .5;
s1 = .002;
r2 = .5;
s2 = .005;
k1s = -r1:s1:r1;
k2s = -r2:s2:r2;
%}
wiggleRoom = .15;
stepSize = .025;
kspace = generateDistortionSearchSpace(wiggleRoom, stepSize);

%% grid search for the best radial distortion parameters
% minNumInliers is used both to (1) stop RANSAC and (2) be a threshold
% above which inlier sets are conidered in the error function
minNumInliers = 30;
for kix=1:size(kspace,1)
    ks = kspace(kix,:);
    k1 = ks(1);
    k2 = ks(2);
    %{
            for k1ix=1:size(k1s,2)
                for k2ix=1:size(k2s,2)
                    k1 = k1s(k1ix);
                    k2 = k2s(k2ix);
    %}
    % before doing ANYTHING else, we need to undistort the image according
    % to these radial distortion coefficients, and then we can proceed with
    % the usual translation estimate procedure
    pin = undistortRadially(pinorig, k1, k2);

    %% find spec centroids in image
    try
        tform = getTransform(P, pin);
    catch e
        continue
    end
    % take the already known imageCentroids and undistort them
    imageCentroids = undistortRadially(imageCentroidsOrig, k1, k2);
    imageCentroidsKs(kix,:,:) = imageCentroids;
    out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
    canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];
    
    
    %% match canonical centroids to those in the characterization
    K = 10;
    [idx, specDistsCanonical] = knnsearch(knownCanonicalCentroids, canonicalCentroids,...
                                 'K', K, 'Distance', 'euclidean');
    % only consider specs whose match is within .xx millimeters
    %closeEnough = .3;
    %specIdxs = idx(specDistsCanonical<closeEnough);
    %specPos = knownCanonicalCentroids(idx,:);
    specPos = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specPos(ix,:,:) = knownCanonicalCentroids(idx(ix,:),:);
    end
    
    %% get all spec normals and maximum brightnesses
    specNormals = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specNormals(ix,:,:) = allSpecNormals(idx(ix,:),:);
    end
    
    %% reflect rays from light off specs
    %reflect the ten nearest specs for each spec found
    R = zeros(size(specPos));
    for ix=1:K
        % compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
        % where Li is normalized vector from spec to light
        %       Ni is normalized normal vector
        %       Ri is normalized reflected vector
        L = (lightPos - reshape(specPos(:,ix,:),size(specPos,1),...
            size(specPos,3))) ./ vecnorm(lightPos - reshape(specPos(:,ix,:),size(specPos,1),size(specPos,3)), 2, 2);
        R(:,ix,:) = L - 2 * dot(L, reshape(specNormals(:,ix,:),...
            size(specNormals,1),size(specNormals,3)), 2) .* reshape(specNormals(:,ix,:),...
            size(specNormals,1),size(specNormals,3));
        R(:,ix,:) = -1.*R(:,ix,:);
    end
    
    %% find a good translation estimate NOT using a RANSAC approach
    % and instead by searching over candidate camera positions in some
    % volume (box) in the world
    % reminiscent of a hough (rhymes with rough) transform
    x0 = 0;
    y0 = 200;
    z0 = 400;
    xf = x0 + 300;
    yf = y0 + 300;
    zf = z0 + 300;
    step = 10; % 1 centimeter
    x = x0:step:xf;
    y = y0:step:yf;
    z = z0:step:zf;
    [X,Y,Z] = ndgrid(x,y,z);

    %%%%%%%%%%%%%%%%%%%%%%%
    votes = empty thing like box
    for spec reflected ray

    end
    %%%%%%%%%%%%%%%%%%%%%%%
    
    %{
    %% find a good translation estimate using a RANSAC approach
    inlierThreshold = 15; % (mm) a reflected ray is an inlier
                          % with respect to a hypothesized camera position
                          % if it pases within 10 millimeters of that 
                          % camera position
    mostInliersSpecPos = [];
    mostInliersR = [];
    minPropInliers = .6; % this threshold means that 60% of the rays must be 
                            % found to be inliers for the model to be used
                            % and this is the stopping condition for the search
                            % for the inliers set
    counter = 0;
    while true
        counter = counter + 1;
        if counter >= 500
            % while the proportion is good, we don't want to waste time
            % looking for a large inliers set when there isn't one for
            % some very wrong radial distortion coefficients
            break
        end
        % hypothesize a possible pair of inliers
        idxsRandomTwo = randi(size(R,1),1,2);
        %k = 1; %when hypothesizing, just take the neareast neighbor specs
        %if nearestDistLines(specPos(idxsRandomTwo,1,:), R(idxsRandomTwo,1,:)) > 2 * inlierThreshold
        %    continue
        %end
        % find the corresponding candidate camera position
        points = reshape(specPos(idxsRandomTwo,1,:),2,3);
        directions = reshape(R(idxsRandomTwo,1,:),2,3);
        [candidate,randomTwoDists] = nearestPointManyLines(points, points+directions);
        if sum(randomTwoDists) > 2 * inlierThreshold
            continue
        end

        % in fact, let's just not let something be 10cm in front of
        % glitter because tons of rays intersect there
        if candidate(3) < 100
            continue
        end
        % sometimes the lines cross near each other behind the glitter plane...
        if candidate(3) < 0
            continue
        end
        % show camera estimated position as a dot: (dots=cameras)
        % check how many of the other rays are inliers for this hypothesized
        % camera positionl; compute dist from hypothesized cam pos to rays
        rayDists = [];
        for k=1:size(R,2)%(size(R,2)=K)
            for ix=1:size(R,1)
                rayDists(ix,k) = distPointToLine(candidate, reshape(specPos(ix,k,:),1,3), reshape(R(ix,k,:),1,3));
            end
        end
        % a single spec doesn't get multiple reflected rays, so we just
        % consider the best reflected ray, assuming that that one is the
        % correct one for this model probably
        [minDists,kmin] = min(rayDists, [], 2);
        inlierIdxs = find(minDists<=inlierThreshold);
        numInliers(counter) = size(inlierIdxs,1);
        kmin = kmin(minDists<=inlierThreshold);
        overallInlierIdxs = [inlierIdxs kmin];
        inliersSpecPos = [];
        inliersImageSpecPos = [];
        inliersR = [];
        inliersMaxBrightness = [];
        for ix=1:size(inlierIdxs)
            inliersImageSpecPos(ix,:) = imageCentroids(inlierIdxs(ix),:);
            inliersSpecPos(ix,:) = specPos(inlierIdxs(ix),kmin(ix),:);
            inliersSpecDistsCanonical(ix) = specDistsCanonical(inlierIdxs(ix),kmin(ix));
            inliersR(ix,:) = R(inlierIdxs(ix),kmin(ix),:);
            %inliersMaxBrightness(ix) = maxBrightness(inlierIdxs(ix),kmin(ix));
        end
        if size(inliersR,1) > size(mostInliersR,1)
            mostInliersImageSpecPos = inliersImageSpecPos;
            mostInliersR = inliersR;
            mostInliersSpecPos = inliersSpecPos;
            mostInliersIdxs = inlierIdxs;
            mostInliersKmin = kmin;
            mostInliersMinDists = minDists;% refers to distance from reflected rays to cam position
            %mostInliersMaxBrightness = inliersMaxBrightness;
            mostInliersSpecDists = inliersSpecDistsCanonical;
        end
        %if size(inliersR,1) > minNumInliers%minPropInliers * size(R,1)
        %    % break from the loop once sufficiently many inliers are found
        %    break
        %end
    end
    %}

    %{
    %% now just for the model with the most inliers, we compute the translation estimate
    if size(mostInliersSpecPos) == 0
        camPosEstLinear = [0 0 0];% nonsense
    else
        camPosEstLinear = nearestPointManyLines(mostInliersSpecPos, mostInliersSpecPos+mostInliersR);
    end
    %}

    % record results for the current pair of radial distortion
    % coefficients, k1 and k2
    numInliersByRadialsCoefs(kix) = size(mostInliersSpecPos,1);
    if numInliersByRadialsCoefs(kix) < minNumInliers
        distsByRadialsCoefs(kix,1:minNumInliers) = nan;%not a valid solution if not enough inliers were found
    else
        % if we only average the best minNumInliers spec distances, then we
        % need to first sort mostInliersSpecsDists in increasing order
        mostInliersSpecDists = sort(mostInliersSpecDists,2,'ascend');
        distsByRadialsCoefs(kix,1:minNumInliers) = mostInliersSpecDists(1:minNumInliers);
    end
end

%% now show the average distances for spec matches in search subspace that was checked
sums = sum(distsByRadialsCoefs,2);
for kix=1:size(kspace)
    x(kix) = kspace(kix,1);
    y(kix) = kspace(kix,2);
    z(kix) = sums(kix) ./ minNumInliers;
end
figure;
%plot3(x,y,z,'go');
%mesh(x,y,z);
k1s = reshape(kspace(:,1),size(kspace,1),1);
k2s = reshape(kspace(:,2),size(kspace,1),1);
%mesh(k2s, k1s, sums); hold on;
plot3(x,y,z,'go','MarkerSize',12);
xlabel('k2');
ylabel('k1');
zlabel('average distance (inliers to characterized, mm)');
%% similar mesh for number of inliers
figure;
%mesh(k2s, k1s, numInliersByRadialsCoefs);
plot3(k2s, k1s, numInliersByRadialsCoefs,'r+');
xlabel('k2');
ylabel('k1');
zlabel('number of inliers');
%% for the best distortion coefficients, show the original 
% imageCentroids as well as the undistorted image centroids so that we 
% can visualize how much they are 'moving' for these distortion 
% coefficients
figure;
tiledlayout(3,3);
for kix=1:size(kspace,1)
    nexttile;
    ks = kspace(kix,:);
    k1 = ks(1);
    k2 = ks(2);
    title([num2str(k1) '  and   ' num2str(k2)]); hold on;
    imagesc(im); colormap(gray); hold on;
    plot(imageCentroidsOrig(:,1),imageCentroidsOrig(:,2),'cx','MarkerSize',12,'LineWidth', 1.5); 
    plot(reshape(imageCentroidsKs(kix,:,1),size(imageCentroidsKs,2),1),reshape(imageCentroidsKs(kix,:,2),size(imageCentroidsKs,2),1),'g+','MarkerSize',12,'LineWidth', 1.5);
end

%% now show the average distance 
