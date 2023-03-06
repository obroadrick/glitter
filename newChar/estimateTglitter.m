% find translation from glitter to camera using a known light 
% source, a picture of sparkling glitter, and a known glitter
% characterization

function [camPosEst, mostInliersSpecPos, mostInliersImageSpecPos, other] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage, skew, other)
    %P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    P = getMar4charPaths();

    % read in image
    if ~exist('ambientImage','var') || (ambientImage == -1)
        im = rgb2gray(imread(impath));
    else
        im = max(rgb2gray(imread(impath)) - ambientImage, 0);
    end

    %% find spec centroids in image
    figure;
    testimpath=impath;
    imagesc(rgb2gray(imread(testimpath)));colormap(gray);hold on;
    plot(pin(:,1),pin(:,2),'rx','MarkerSize',15);
    [transformPath, ~, ~] = getCheckerboardHomography(expdir, impath);
    tform = matfile(transformPath).tform;
    options.filter = true;
    [imageCentroids,~,intensitys] = singleImageFindSpecs(im, 20, options); %normal use
    %[imageCentroids,~] = singleImageFindSpecsNoFilter(im); % special use
    plot(imageCentroids(:,1),imageCentroids(:,2),'b+');
    out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
    canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];
    idx = cleanPoints(canonicalCentroids, 18.25, .15);
    canonicalCentroids = canonicalCentroids(idx,:);
    fprintf([int2str(size(canonicalCentroids,1)) ' specs found; ']);

    intensitys = intensitys(:,idx);
    %% match canonical centroids to those in the characterization
    knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
    K = 10;
    [idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids,...
                                 'K', K, 'Distance', 'euclidean');
    specPos = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specPos(ix,:,:) = knownCanonicalCentroids(idx(ix,:),:);
    end
    % draw vector map where each vector goes from a spec to its matched 
    % spec neighbor
    figure;
    quiver(canonicalCentroids(:,1), canonicalCentroids(:,2),...
        knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1),...
        knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2),...
        'off',...%specifies to not automatically scale the vector lengths
        'LineWidth',2);
    title('vectors from seen specs to matched specs (in glitter coords)')

    x = knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1);
    y = knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2);
    figure;
    histogram(atan2(y, x),'NumBins',10);
    title('directions from seen spec to matched spec (in glitter coords)');
    xlabel('direction in radians');
    
    %% Get the specs' surface normals
    if isfield(other, "customNormals")
        allSpecNormals = matfile(other.customNormals).specNormals;
    else
        allSpecNormals = matfile(P.specNormals).specNormals;
    end
    specNormals = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specNormals(ix,:,:) = allSpecNormals(idx(ix,:),:);
    end
    usingMaxBrightness = false;
    if usingMaxBrightness 
    allMaxBrightness = matfile(P.maxBrightness).maxBrightness;
    maxBrightness = zeros(size(idx,1),K);
    for ix=1:size(idx,1)
        maxBrightness(ix,:) = allMaxBrightness(idx(ix,:));
    end
    end
    
    %% find dist of ray to pinhole for all specs
    % compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
    % where Li is normalized vector from spec to light
    %       Ni is normalized normal vector
    %       Ri is normalized reflected vector
    % reflect light off all the specs
    allSpecPos = knownCanonicalCentroids;
    for ix=1:size(allSpecPos)
        L = -((lightPos - allSpecPos(ix,:)) ./ vecnorm(lightPos - allSpecPos(ix,:),2, 2));
        %disp(norm(L));
        %disp(norm(allSpecNormals(ix,:)));
        allR(ix,:) = L - (2 * dot(L, allSpecNormals(ix,:), 2)) * allSpecNormals(ix,:);
        %allR(ix,:) = allR(ix,:);
    end
    
    
    %{
    % as a major sanity check: draw a single incoming ray, surface normal, 
    % and reflected ray
    figure;
    ix = [1];
    L = ((lightPos - allSpecPos(ix,:)) ./ vecnorm(lightPos - allSpecPos(ix,:),2, 2));
    % light ray in blue
    plot3([allSpecPos(ix,1) allSpecPos(ix,1)+L(ix,1)], [allSpecPos(ix,2) allSpecPos(ix,2)+L(ix,2)], [allSpecPos(ix,3) allSpecPos(ix,3)+L(ix,3)],'color','blue');
    hold on;
    xlabel('x');ylabel('y');zlabel('z');
    % surface normal in green
    plot3([allSpecPos(ix,1) allSpecPos(ix,1)+allSpecNormals(ix,1)], [allSpecPos(ix,2) allSpecPos(ix,2)+allSpecNormals(ix,2)], [allSpecPos(ix,3) allSpecPos(ix,3)+allSpecNormals(ix,3)],'color','green');
    % reflected ray in red
    plot3([allSpecPos(ix,1) allSpecPos(ix,1)+allR(ix,1)], [allSpecPos(ix,2) allSpecPos(ix,2)+allR(ix,2)], [allSpecPos(ix,3) allSpecPos(ix,3)+allR(ix,3)],'color','red');
    %}

    %%
    %compute distances to pinhole for these allR reflected rays
    
    if other.compare
    if skew
        knownCamPos = matfile([expdir 'camPosSkew.mat']).camPos; 
    else
        knownCamPos = matfile([expdir 'camPos.mat']).camPos; 
    end
    end
    if other.compare
    allTrueDists = [];
    for ix=1:size(allR,1)
        allTrueDists(ix) = distPointToLine(knownCamPos, allSpecPos(ix,:), allR(ix,:));
    end
    end
    
    %%
    % also get the brightness of those specs
    %brightness = interp2(double(im), imageCentroids(:,1),
    %imageCentroids(:,2));% linearly interpolate to get brightness of spec
    brightness = interp2(double(im), imageCentroids(:,1), imageCentroids(:,2));
    
    %{
    % also show the original max image so that we can compare the centroids
    % that we found with the centroids that we characterized from the start
    % now show the canonical spec centroids (mapped onto this image coordinate
    % system using the inverse homography) to show them 
    tforminv = invert(tform);
    characterizedCentroidsOnThisImage = transformPointsForward(tforminv, [knownCanonicalCentroids(:,1) knownCanonicalCentroids(:,2)]);
    allTrueDistsNormalized = exp(-allTrueDists.^2/50);
    %allTrueDistsNormalized = (allTrueDists - min(allTrueDists)) / max(allTrueDists);
    red = [1 0 0];
    green = [0 1 0];
    colors = [];
    for ix=1:size(allTrueDistsNormalized,2)
        colors(ix,:) = green + (red-green)*allTrueDistsNormalized(ix);
    end
    %}

    %%
    %disp('now plotting');
    %{
    % the big/dense 2-tile plot
    figure;
    tiledlayout(1,2);colormap(gray);
    ax3 = nexttile;
    imagesc(im); hold on;
    scatter(characterizedCentroidsOnThisImage(:,1),characterizedCentroidsOnThisImage(:,2),18,colors,'filled');
    plot(imageCentroids(:,1),imageCentroids(:,2),'cx','MarkerSize',12,'LineWidth', 3);
    maxImagePath = P.maxImage;
    maxImage = imread(maxImagePath);
    knownImageCentroids = matfile(P.imageCentroids).imageCentroids;
    ax4 = nexttile;
    imagesc(maxImage); hold on;
    plot(knownImageCentroids(:,1),knownImageCentroids(:,2),'r+');
    linkaxes([ax3 ax4]);
    %}
    
    %% reflect rays from light off specs
    %reflect the ten nearest specs for each spec found
    R = zeros(size(specPos));
    L = zeros(size(specPos));
    for ix=1:K
        % compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
        % where Li is normalized vector from spec to light
        %       Ni is normalized normal vector
        %       Ri is normalized reflected vector
        Lcur = (lightPos - reshape(specPos(:,ix,:),size(specPos,1),...
            size(specPos,3))) ./ vecnorm(lightPos - reshape(specPos(:,ix,:),size(specPos,1),size(specPos,3)), 2, 2);
        L(:,ix,:) = Lcur;
        R(:,ix,:) = Lcur - 2 * dot(Lcur, reshape(specNormals(:,ix,:),...
            size(specNormals,1),size(specNormals,3)), 2) .* reshape(specNormals(:,ix,:),...
            size(specNormals,1),size(specNormals,3));
        R(:,ix,:) = -1.*R(:,ix,:);
    end

    % find a good translation estimate using a RANSAC approach
    rng(314159);
    %inlierThreshold = 50; % (mm) a reflected ray is an inlier
                          % with respect to a hypothesized camera position
                          % if it pases within xx millimeters of that 
                          % camera position
                          % draw rig
    inlierThreshold = other.inlierThreshold;
    makeBigFigure = false;
    if makeBigFigure
        GLIT_TO_MON_PLANES = 424;
        GLIT_TO_MON_EDGES_X = 178;
        GLIT_TO_MON_EDGES_Y = 88.8 + 77.1 + 8*18.25;
        M = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,GLIT_TO_MON_EDGES_Y);
        figure;
        legendItems = [];hold on;
        % table: (made up coords, doesn't matter, mostly for fun)
        tx = [-400 -400 600 600];
        ty = -2.*[-120 -120 -120 -120];
        tz = -1.*[-250 1000 1000 -250];
        tc = ['k'];
        legendItems(1) = patch(tx,ty,tz,tc,'DisplayName','Table');
        % shown known ground truth camera position
        legendItems(size(legendItems,2)+1) = scatter3(knownCamPos(1),knownCamPos(2),knownCamPos(3),100,'blue','filled','o','DisplayName','True Camera');
        % show light source as a dot
        legendItems(size(legendItems,2)+1) = scatter3(lightPos(1),lightPos(2),lightPos(3),'filled','DisplayName','Light');
        % draw all lines red to start
        Rlong = R .* 1000;
        for ix=1:size(Rlong,1)
            % draw reflected rays
            x = [specPos(ix,1,1) specPos(ix,1,1)+Rlong(ix,1,1)]';
            y = [specPos(ix,1,2) specPos(ix,1,2)+Rlong(ix,1,2)]';
            z = [specPos(ix,1,3) specPos(ix,1,3)+Rlong(ix,1,3)]';
            color = [1 0 0];
        end
        legPos = size(legendItems,2)+1;
        camroll(-80);
    end
    numInliers = [];
    mostInliersSpecPos = [];
    mostInliersR = [];
    mostInliersL = [];
    mostInliersIntensitys = [];
    numRansacIters = 10000;
    for counter=1:numRansacIters %this is constant right now but could (should) be more dynamic/reactive than that
        
        % hypothesize a possible pair of inliers
        idxsRandomTwo = randi(size(R,1),1,2);
        %when hypothesizing, just take the neareast neighbor specs
        %{
        k1 = randi(size(specPos,2));
        k2 = randi(size(specPos,2));
        if nearestDistLines(specPos(idxsRandomTwo,k1,:), R(idxsRandomTwo,k2,:)) > 2 * inlierThreshold
            continue
        end
        %}
        
        inlierPairFound = false;
        inlierPairK = -1;
        minFound = 10000000;
        for k1=1:size(specPos,2)
            for k2=1:size(specPos,2)
                d = nearestDistLines(specPos(idxsRandomTwo,k1,:), R(idxsRandomTwo,k2,:));
                if  d < 2 * inlierThreshold
                    if d < minFound
                        minFound = d;
                        inlierPairFound = true;
                        inlierPairK1 = k1;
                        inlierPairK2 = k2;
                    end
                end
            end
        end
        if ~inlierPairFound
            continue
        end
        
        % find the corresponding candidate camera position
        points = specPos(idxsRandomTwo,inlierPairK1,:);
        directions = R(idxsRandomTwo,inlierPairK2,:);
        candidate = pointBetweenLines(points, directions);
        % sometimes the lines cross near each other behind the glitter plane...
        % but also a position within a few cm in front of the plane is not
        % cool... call it 250mm=25cm
        candidateThreshold = -445;
        if candidate(3) > candidateThreshold
            continue
        end
        % first just check whether this hypothesized camera position even
        % keeps the two rays which hypothesized it as inliers (AKA are the
        % two rays we randomly chose even close together?) and if they are
        % not, move on to the next pair
        if distPointToLine(candidate, reshape(specPos(idxsRandomTwo(1),1,:),1,3), reshape(R(idxsRandomTwo(1),1,:),1,3)) > inlierThreshold
            %disp('here')%based on little testing, this doesn't affect
            %accuracy of method (aka we weren't badly relying on far apart
            %rays to give the right middle spot (even tho they might do so
            %on average ish))
            continue
        end

        % show camera estimated position as a dot: (dots=cameras)
        cam=candidate;
        %legendItems(legPos) = scatter3(cam(1),cam(2),cam(3),25,'red','o','filled','DisplayName','Hypothesized Camera');
        % check how many of the other rays are inliers for this hypothesized
        % camera position
        % compute dist from hypothesized cam pos to rays
        dists = [];
        for k=1:size(R,2)%(size(R,2)=K)
            for ix=1:size(R,1)
                dists(ix,k) = distPointToLine(candidate, reshape(specPos(ix,k,:),1,3), reshape(R(ix,k,:),1,3));
            end
        end
        % a single spec doesn't get multiple reflected rays, so we just
        % consider the best reflected ray, assuming that that one is the
        % correct one for this model probably
        [minDists,kmin] = min(dists, [], 2);
        inlierIdxs = find(minDists<=inlierThreshold);
        numInliers(counter) = size(inlierIdxs,1);
        kmin = kmin(minDists<=inlierThreshold);
        overallInlierIdxs = [inlierIdxs kmin];
        inliersSpecPos = [];
        inliersImageSpecPos = [];
        inliersR = [];
        inliersL = [];
        inliersMaxBrightness = [];
        inliersIntensitys = [];
        for ix=1:size(inlierIdxs)
            inliersImageSpecPos(ix,:) = imageCentroids(inlierIdxs(ix),:);
            inliersIntensitys(ix,:) = intensitys(inlierIdxs(ix));
            inliersSpecPos(ix,:) = specPos(inlierIdxs(ix),kmin(ix),:);
            inliersR(ix,:) = R(inlierIdxs(ix),kmin(ix),:);
            inliersL(ix,:) = L(inlierIdxs(ix),kmin(ix),:);
            if usingMaxBrightness
            inliersMaxBrightness(ix) = maxBrightness(inlierIdxs(ix),kmin(ix));
            end
        end
        if size(inliersR,1) > size(mostInliersR,1)
            mostInliersImageSpecPos = inliersImageSpecPos;
            mostInliersR = inliersR;
            mostInliersL = inliersL;
            mostInliersSpecPos = inliersSpecPos;
            mostInliersIdxs = inlierIdxs;
            mostInliersKmin = kmin;
            mostInliersMinDists = minDists;
            mostInliersMaxBrightness = inliersMaxBrightness;
            mostInliersIntensitys = inliersIntensitys;
        end
        if makeBigFigure
            % now show the two lines
            for i=1:size(idxsRandomTwo,2)
                ix = idxsRandomTwo(i);
                % draw reflected rays
                x = [specPos(ix,1,1) specPos(ix,1,1)+Rlong(ix,1,1)]';
                y = [specPos(ix,1,2) specPos(ix,1,2)+Rlong(ix,1,2)]';
                z = [specPos(ix,1,3) specPos(ix,1,3)+Rlong(ix,1,3)]';
                color = [0 0 1];
                line(x,y,z,'Color',color);
            end
            % and their inliers as well
            inliersR = inliersR * 1000;
            for ix=1:size(inliersR,1)
                % draw reflected rays
                x = [inliersSpecPos(ix,1) inliersSpecPos(ix,1)+inliersR(ix,1)]';
                y = [inliersSpecPos(ix,2) inliersSpecPos(ix,2)+inliersR(ix,2)]';
                z = [inliersSpecPos(ix,3) inliersSpecPos(ix,3)+inliersR(ix,3)]';
                color = [0 1 0];
                line(x,y,z,'Color',color);
            end
            view([-110 -30]);
            %camroll(-80);
            daspect([1 1 1]);
            curNumInliers = numInliers(size(numInliers,2));
            title(['Number of inliers: ' num2str(curNumInliers) ' (max so far: ' num2str(max(numInliers)) ')']);
            %pause(.5);%
            red = [1 0 0];
            % now go back over and draw them all back to red
            for i=1:size(idxsRandomTwo,1)
                ix = idxsRandomTwo(i);
                % draw reflected rays
                x = [specPos(ix,1,1) specPos(ix,1,1)+Rlong(ix,1,1)]';
                y = [specPos(ix,1,2) specPos(ix,1,2)+Rlong(ix,1,2)]';
                z = [specPos(ix,1,3) specPos(ix,1,3)+Rlong(ix,1,3)]';
                line(x,y,z,'Color',red);
            end
            % and their inliers as well
            for ix=1:size(inliersR,1)
                % draw reflected rays
                x = [inliersSpecPos(ix,1) inliersSpecPos(ix,1)+inliersR(ix,1)]';
                y = [inliersSpecPos(ix,2) inliersSpecPos(ix,2)+inliersR(ix,2)]';
                z = [inliersSpecPos(ix,3) inliersSpecPos(ix,3)+inliersR(ix,3)]';
                line(x,y,z,'Color',red);
            end
        end
    end
    
    fprintf([num2str(max(numInliers)) ' inliers; ']);
    avgK = sum(mostInliersKmin)/size(mostInliersKmin,1);
    stdK = std(mostInliersKmin);
    avgD = sum(dist(mostInliersIdxs))/size(mostInliersKmin,1);
    stdD = std(dist(mostInliersIdxs));
    fprintf('avg. matches are %.2fth nearest neighbors (std=%f), at distance %f mm (std=%f)\n', avgK, stdK, avgD, stdD);

    %%
    % now just for the model with the most inliers, we build up a 
    % camera position estimate
    godlyStd = 3.5;%based loosely on looking at Addy's receptive field-probing results
    quickEst = nearestPointManyLines(mostInliersSpecPos, mostInliersSpecPos+mostInliersR);
    if isfield(other,'quickEstimate') && other.quickEstimate == true
        % just return the estimate based on the nearestPointManyLines
        camPosEst = quickEst;
        mostInliersSpecPos = mostInliersSpecPos;
        mostInliersImageSpecPos = mostInliersImageSpecPos;
        other.mostInliersL = mostInliersL;
        other.mostInliersSpecPos = mostInliersSpecPos;
        other.mostInliersIdxs = mostInliersIdxs;
        other.lightPos = lightPos;
        other.mostInliersR = mostInliersR;
        other.overallIdx = idx;
        other.mostInliersKmin = mostInliersKmin;

        other.mostInliersIntensitys = mostInliersIntensitys;
    
        other.specPos = canonicalCentroids;

        % also return the distance of each reflected ray to the estimated
        % pinhole position for aperture things
        reflectedRayToPinholeDists = [];
        for i=1:size(mostInliersR,1)
            reflectedRayToPinholeDists(i) = distPointToLine(camPosEst, reshape(mostInliersSpecPos(i,:),1,3), reshape(mostInliersR(i,:),1,3));
        end
        other.reflectedRayToPinholeDists = reflectedRayToPinholeDists;
    end
    if ~(isfield(other,'quickEstimate') && other.quickEstimate == true)
    x0 = [quickEst godlyStd 255];
    more.charSpecPos = matfile([P.canonicalCentroids]);
    more.charSpecNormals = matfile([P.specNormals]);
    errFun = @(x) lossFunc(x(1:3), mostInliersSpecPos, mostInliersR, mostInliersIntensitys, x(4), x(5), more);
    options = optimset('PlotFcns',@optimplotfval);
    xf = fminsearch(errFun, x0, options);
    camPosEst = xf(1:3);
    godlyStdFound = xf(4);
    godlyPeakFound = xf(5);
    fprintf('Sparkle Gaussian stand dev (dist to pinhole, mm): %f and height (intensity, 0-255): %f\n',godlyStdFound, godlyPeakFound);

    % also compute dists to the known camera location for all shiny specs
    trueDists = [];
    for k=1:size(R,2)%size(R,2) is also just K from knnsearch above
        for ix=1:size(R,1)
            trueDists(ix,k) = distPointToLine(knownCamPos, reshape(specPos(ix,k,:),1,3), reshape(R(ix,k,:),1,3));
        end
    end
    % let's have a look at the nearest to correct rays for each spec centroid
    [minTrueDists,kmin] = min(trueDists, [], 2);
    inlierIdxs = [1:size(minTrueDists,1)]';
    numInliers(counter) = size(inlierIdxs,1);
    %kmin = kmin(minTrueDists<=inlierThreshold);
    overallInlierIdxs = [inlierIdxs kmin];
    bestSpecPos = [];
    bestR = [];
    bestMaxBrightness = [];
    for ix=1:size(inlierIdxs,1)
        bestSpecPos(ix,:) = specPos(inlierIdxs(ix),kmin(ix),:);
        bestR(ix,:) = R(inlierIdxs(ix),kmin(ix),:);
        if usingMaxBrightness
            bestMaxBrightness(ix) = maxBrightness(inlierIdxs(ix),kmin(ix));
        end
    end
    %FROMHERE
    % we can also get these distances for just inliers with respect to the 
    % true camera position
    trueInlierIdxs = find(minTrueDists<=inlierThreshold);
    numInliers(counter) = size(trueInlierIdxs,1);
    kmin = kmin(minTrueDists<=inlierThreshold);
    overallInlierIdxs = [trueInlierIdxs kmin];
    trueInlierSpecPos = [];
    trueInlierR = [];
    trueInlierMaxBrightness = [];
    for ix=1:size(trueInlierIdxs,1)
        trueInlierSpecPos(ix,:) = specPos(trueInlierIdxs(ix),kmin(ix),:);
        trueInlierR(ix,:) = R(trueInlierIdxs(ix),kmin(ix),:);
        if usingMaxBrightness
            trueInlierMaxBrightness(ix) = maxBrightness(trueInlierIdxs(ix),kmin(ix));
        end
    end
    %TOHERE
    % we can also get these "true" distances for just the inliers from RANSAC
    trueDistsInliers = [];
    for k=1:size(R,2)%size(R,2) is also just K from knnsearch above
        for ix=1:size(mostInliersSpecPos,1)
            trueDistsInliers(ix,k) = distPointToLine(knownCamPos, reshape(mostInliersSpecPos(ix,:),1,3), reshape(mostInliersR(ix,:),1,3));
        end
    end
    [minTrueDistsInliers,kmin] = min(trueDistsInliers, [], 2);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    % draw rig
    figure;
    %{
    % glitter square:
    gx = [0 M.GLIT_WIDTH M.GLIT_WIDTH 0]; 
    gy = [0 0 M.GLIT_HEIGHT M.GLIT_HEIGHT]; 
    gz = [0 0 0 0];
    gc = ['b'];
    legendItems = [];
    legendItems(1) = patch(gx,gy,gz,gc,'DisplayName', 'Glitter');hold on;
    % monitor:
    mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
    my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
    mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
    mc = ['g'];
    %}
    %legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
    % table: (made up coords, doesn't matter, mostly for fun)
    legendItems = [];
    hold on;
    tx = [-400 -400 600 600];
    ty = 3.*[120 120 120 120];
    tz = -1.*[-250 1000 1000 -250];
    tc = ['k'];
    legendItems(1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    % show camera as a dot: (dots=cameras)
    cam=camPosEst;
    legendItems(size(legendItems,2)+1) = scatter3(cam(1),cam(2),cam(3),100,'red','o','filled','DisplayName','Estimated Camera');
    % shown known ground truth camera position
    fprintf('Checker=(%.2f %.2f %.2f) ', knownCamPos);
    fprintf('Sparkle=(%.2f %.2f %.2f) ', camPosEst);
    fprintf('Diff=(%.2f %.2f %.2f) |Diff|=%.2f\n', cam - knownCamPos, norm(cam - knownCamPos));
    legendItems(size(legendItems,2)+1) = scatter3(knownCamPos(1),knownCamPos(2),knownCamPos(3),...
        100,'blue','filled','o','DisplayName','True Camera');
    % show light source as a dot
    legendItems(size(legendItems,2)+1) = scatter3(lightPos(1),lightPos(2),lightPos(3),...
        'filled','DisplayName','Light');
    % draw all the passed lines
    %scale for drawing
    specNormals = specNormals .* 50;
    Rtocam = R*1000;
    %reflectedRaysCamPlane = reflectedRaysCamPlane .* 100;
    % get normalized spec distances for drawing
    %distNormalized = dist / closeEnough;% color code for
    %get characterized max brightnesses
    
    %brightnessNormalized = double(brightness) ./ bestMaxBrightness';% color code for
    for ix=1:size(mostInliersSpecPos,1)
        % draw reflected rays
        x = [mostInliersSpecPos(ix,1) mostInliersSpecPos(ix,1)+mostInliersR(ix,1)*1000]';
        y = [mostInliersSpecPos(ix,2) mostInliersSpecPos(ix,2)+mostInliersR(ix,2)*1000]';
        z = [mostInliersSpecPos(ix,3) mostInliersSpecPos(ix,3)+mostInliersR(ix,3)*1000]';
        %distance form camera estimate
        green = [0 1 0];
        red = [1 0 0];
        %c = min(1, distNormalized(ix)); %color code by spec closeness
        %c = 1.0*min(1, brightnessNormalized(ix)); %color code by spec brightness
        %disp(c);
        color = green;% + c*(red-green);
        %color = [1 c 1];
        line(x,y,z,'Color',color);
    end
    
    title(sprintf('final set of inliers (%d/%d)', size(mostInliersSpecPos,1), size(specPos,1)));
    % set viewpoint:
    view([-110 -30]);
    camroll(-80);
    daspect([1 1 1]);
    legend(legendItems);
    if isfield(other,'quickEstimate') && other.quickEstimate == true
        return;
    end
    
    %
    %%%
    %%%
    %%%%%
    %%%%%%%%
    %%%%%%%%% now plot brightness vs distance to true pinhole
    %%%%%%    
    %%%%%
    %%%
    if usingMaxBrightness
    figure;
    scatter(minTrueDists, brightnessNormalized');
    xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
    ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
    title('Intensity vs Dist to Pinhole: All reflected rays');
    % also do so for just the inliers of the final estimate
    trueDistsInliersOnly = zeros(size(mostInliersR,1),1);
    for ix=1:size(mostInliersR,1)
        trueDistsInliersOnly(ix) = distPointToLine(knownCamPos, reshape(mostInliersSpecPos(ix,:),1,3), reshape(mostInliersR(ix,:),1,3));
    end
    end
    %brightnessNormalizedInliersOnly = brightnessNormalized(mostInliersIdxs);
    %brightnessNormalizedInliersOnly = mostInliersMaxBrightness';
    if usingMaxBrightness
        brightnessNormalizedInliersOnly = brightness(mostInliersIdxs) ./ maxBrightness(mostInliersIdxs);
        figure;
        scatter(minTrueDistsInliers, brightnessNormalizedInliersOnly);
        xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
        ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
        title('Intensity vs Dist to Pinhole: Inliers Only');
    end
    
    %brightnessNormalizedInliersOnly = brightnessNormalized(mostInliersIdxs);
    %brightnessNormalizedInliersOnly = mostInliersMaxBrightness';
    %{
    brightnessNormalizedInliersOnly = brightness(mostInliersIdxs) ./ maxBrightness(mostInliersIdxs);
    figure;
    scatter(minTrueDistsInliers, brightnessNormalizedInliersOnly);
    xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
    ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
    title('Intensity vs Dist to Pinhole: "True" inliers only');
    
    scatter(exp(-minTrueDistsInliers.^2./50), brightnessNormalizedInliersOnly);
    xlabel('Bogus fraction of the ray in the aperture');
    ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
    title('Intensity vs Dist to Pinhole: "True" inliers only');
    %}

    % returns:
    camPosEst = camPosEst;
    mostInliersSpecPos = mostInliersSpecPos;
    mostInliersImageSpecPos = mostInliersImageSpecPos;
    
    other.mostInliersL = mostInliersL;
    other.mostInliersSpecPos = mostInliersSpecPos;
    other.mostInliersIdxs = mostInliersIdxs;
    other.lightPos = lightPos;
    other.mostInliersR = mostInliersR;
    other.overallIdx = idx;
    other.mostInliersKmin = mostInliersKmin;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% functions
function d = nearestDistLines(points, directions)
    % normal for the parallel planes each containing one of the lines
    n = cross(directions(1,:),directions(2,:));
    % now get a point from each of the planes (from the lines)
    p1 = points(1,:);
    p2 = points(2,:);
    % a vector from one of these points to the other can then
    % be projected onto the plane normal to get distance
    % between the planes
    v = p1 - p2;
    proj_n_v = dot(v,n,2) / norm(n)^2 .* n;
    d = norm(proj_n_v);
end
function p = pointBetweenLines(points, directions)
    p1 = points(1,:)';
    p2 = points(2,:)';
    d1 = directions(1,:)';
    d2 = directions(2,:)';
    A = [dot(p1,d1) -1*dot(p1,d2);...
         dot(p2,d1) -1*dot(p2,d2)];
    B = [-1*dot(p1-p2,p1);...
         -1*dot(p1-p2,p2)]; 
    x = linsolve(A,B);
    s = x(1);
    t = x(2);
    Q = p1 + d1.*t;
    R = p2 + d2.*s;
    p = (Q+R)./2;
    p = p';
end
function d = distPointToLine(point, pointOnLine, direction)
    a = pointOnLine';% point on the line
    n = (direction./norm(direction))';% unit vector in direction of line
    p = point';% point whose distance is being computed
    d = norm((p-a)-(dot((p-a),n,1)*n));
end
function error = lossFunc(camPos, mostInliersSpecPos, mostInliersR, mostInliersIntensitys, std, peak, more)
    charSpecPos = more.charSpecPos;
    charSpecNormals = more.charSpecNormals;
    
    % For every characterized spec, reflect a ray from lightPos according
    % to its surface normal, find the distance from the pinhole camPos, and
    % compute its predicted intensity based on our model.
    for ix=1:size(charSpecPos)
        
    end

    error = 0;
    % for each inlier sparkle
    for i=1:size(mostInliersSpecPos,1)
        % compute distance from reflected ray to pinhole (camPos)
        dist = distPointToLine(camPos, reshape(mostInliersSpecPos(i,:),1,3), reshape(mostInliersR(i,:),1,3));
        %disp(dist);
        %{
        % weight this distance's contribution to the loss function by
        % sparkle intensity(first/naive version)
        weight = mostInliersIntensitys(i);
        error = error + dist * weight;
        %}

        % based on this distance, estimate/predict the intensity
        %std = 5;
        predictedDist = sqrt(-2*std^2*log(mostInliersIntensitys(i)/peak));
        %disp(predictedDist);
        %disp(mostInliersIntensitys(i));
        error = error + abs(dist - predictedDist);
    end
    %disp(mostInliersIntensitys);
    % divide by the number of sparkles so that we get a more interpretable
    % loss function value (i.e. the average difference between the observed
    % distance from pinhole and the predicted distance to pinhole based on
    % brightness
    error = error / size(mostInliersIntensitys,1);
end
function error = lossFuncOriginal(camPos, mostInliersSpecPos, mostInliersR, mostInliersIntensitys, std, peak)
    error = 0;
    % for each inlier sparkle
    for i=1:size(mostInliersSpecPos,1)
        % compute distance from reflected ray to pinhole (camPos)
        dist = distPointToLine(camPos, reshape(mostInliersSpecPos(i,:),1,3), reshape(mostInliersR(i,:),1,3));
        %disp(dist);
        %{
        % weight this distance's contribution to the loss function by
        % sparkle intensity(first/naive version)
        weight = mostInliersIntensitys(i);
        error = error + dist * weight;
        %}

        % based on this distance, estimate/predict the intensity
        %std = 5;
        predictedDist = sqrt(-2*std^2*log(mostInliersIntensitys(i)/peak));
        %disp(predictedDist);
        %disp(mostInliersIntensitys(i));
        error = error + abs(dist - predictedDist);
    end
    %disp(mostInliersIntensitys);
    % divide by the number of sparkles so that we get a more interpretable
    % loss function value (i.e. the average difference between the observed
    % distance from pinhole and the predicted distance to pinhole based on
    % brightness
    error = error / size(mostInliersIntensitys,1);
end