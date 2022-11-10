% find translation from glitter to camera using a known light 
% source, a picture of sparkling glitter, and a known glitter
% characterization

function [camPosEst, mostInliersSpecPos, mostInliersImageSpecPos, other] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage)

    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    M = matfile([expdir 'measurements.mat']).M;

    % read in image
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july_characterization/pointLightSource2.JPG';
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july12characterization/pointLightSource/DSC_2202.JPG';
    %impath = [P.characterizationDirectory 'circlesOnMonitor/2022-07-11T16,42,12circle-calib-W1127-H574-S48.jpg'];
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/xenon_06_23_2022/2022-06-23T14,15,20Single-Glitter.JPG';
    %impath = '/Users/oliverbroadrick/Desktop/oliver-took-pictures/homographies and point captures/hilight.JPG';
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july19characterization/circleOnMonitor/2022-07-19T13,54,52circle-calib-W1127-H574-S48.jpg';
    if ~exist('ambientImage','var')
        im = rgb2gray(imread(impath));
    else
        im = max(rgb2gray(imread(impath)) - ambientImage, 0);
    end
    
    % get lighting position in canonical coords form lighting position in
    % monitor pixel coords
    %monitorCoords = [1127 574];
    %x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - M.PX2MM_X * monitorCoords(1); 
    %y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - M.PX2MM_Y * monitorCoords(2); 
    %lightPos = [x y M.GLIT_TO_MON_PLANES];
    %lightPos = [0 (130.1-84.05) 440];
    %lightPos = [0 133.1-72.9 465];
    %lightPos = [0 132.1-72.7 461];
    %lightPos = [0 131.1-72.9 462];
    
    %% find spec centroids in image
    %pin = [1217.34838867 5145.87841797; 1005.55084  295.4278; 6501.5874  490.0575; 6501.952 5363.594];
    %pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y 
    %pin = [1642.2677 5380.783; 1337.9928 733.52966; 6572.239 726.0792; 6226.173 5270.477];
    %{
    allPts = matfile(P.characterizationPoints).arr;
    pin = allPts(1,:);
    pinx = [pin{1}(1) pin{2}(1) pin{3}(1) pin{4}(1)];
    piny = [pin{1}(2) pin{2}(2) pin{3}(2) pin{4}(2)];
    pin = double([pinx' piny']);
    %}
    %{
    pin = [850.0531005859375	4638.21875;...
            454.743408203125	503.7138366699219;...
            7711.8046875	540.760009765625;...
            7277.14111328125	4664.25];
    %}
    %pin = [865.933837890625	4639.2392578125; 473.364990234375	505.5672302246094; 7731.4736328125	541.7628173828125; 7294.72216796875	4668.791015625];
    
    figure;
    %testimpath = "/Users/oliverbroadrick/Desktop/glitter-stuff/july_characterization/homography images/DSC_1931.JPG";
    %testimpath = [P.homographyImages 'DSC_2192.JPG'];
    %testimpath = '/Users/oliverbroadrick/Desktop/glitter-stuff/july12characterization/pointLightSource/DSC_2202.JPG';
    %testimpath = [P.characterizationDirectory 'circlesOnMonitor/2022-07-11T16,42,12circle-calib-W1127-H574-S48.jpg'];
    %testimpath = '/Users/oliverbroadrick/Desktop/oliver-took-pictures/homographies and point captures/DSC_2538.JPG';
    %testimpath = '/Users/oliverbroadrick/Desktop/oliver-took-pictures/homographies and point captures/verybright.JPG';
    testimpath=impath;
    imagesc(rgb2gray(imread(testimpath)));colormap(gray);hold on;
    plot(pin(:,1),pin(:,2),'rx','MarkerSize',15);
    tform = getTransform(P, pin);
    [imageCentroids,~] = singleImageFindSpecs(im); %normal use
    %[imageCentroids,~] = singleImageFindSpecsNoFilter(im); % special use
    disp([int2str(size(imageCentroids,1)) ' specs detected...']);
    %disp(size(imageCentroids));
    plot(imageCentroids(:,1),imageCentroids(:,2),'b+');
    out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
    canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];
    
    %% match canonical centroids to those in the characterization
    knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
    K = 15;
    [idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids,...
                                 'K', K, 'Distance', 'euclidean');
    % only consider specs whose mwhatch is within .xx millimeters
    %closeEnough = .3;
    %specIdxs = idx(dist<closeEnough);
    %specPos = knownCanonicalCentroids(idx,:);
    specPos = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specPos(ix,:,:) = knownCanonicalCentroids(idx(ix,:),:);
    end
    % draw vector map where each vector goes from a spec to its nearest spec
    % neighbor
    figure;
    quiver(canonicalCentroids(:,1), canonicalCentroids(:,2),...
        knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1),...
        knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2),...
        'LineWidth',2);


    x = knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1);
    y = knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2);
    figure;
    histogram(atan2(y, x));
    title('histogram of vector directions');
    xlabel('direction in radians');
    
    %%
    allSpecNormals = matfile(P.specNormals).specNormals;
    specNormals = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specNormals(ix,:,:) = allSpecNormals(idx(ix,:),:);
    end
    allMaxBrightness = matfile(P.maxBrightness).maxBrightness;
    maxBrightness = zeros(size(idx,1),K);
    for ix=1:size(idx,1)
        maxBrightness(ix,:) = allMaxBrightness(idx(ix,:));
    end
    
    %% NEW CODE TO FIND THE DIST OF RAY TO PINHOLE FOR ALL SPECS SO WE CAN VISUALIZE WHICH SPECS SEEM GOOD
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
    % as a major sanity check:
    % draw a single incoming ray, surface normal, and computer reflected
    % ray
    % normal for it
    figure;
    ix = [1];
    L = ((lightPos - allSpecPos(ix,:)) ./ vecnorm(lightPos - allSpecPos(ix,:),2, 2));
    plot3([allSpecPos(ix,1) allSpecPos(ix,1)+L(ix,1)], [allSpecPos(ix,2) allSpecPos(ix,2)+L(ix,2)], [allSpecPos(ix,3) allSpecPos(ix,3)+L(ix,3)]);
    hold on;
    plot3([allSpecPos(ix,1) allSpecPos(ix,1)+allSpecNormals(ix,1)], [allSpecPos(ix,2) allSpecPos(ix,2)+allSpecNormals(ix,2)], [allSpecPos(ix,3) allSpecPos(ix,3)+allSpecNormals(ix,3)],'color','green');
    
    plot3([allSpecPos(ix,1) allSpecPos(ix,1)+allR(ix,1)], [allSpecPos(ix,2) allSpecPos(ix,2)+allR(ix,2)], [allSpecPos(ix,3) allSpecPos(ix,3)+allR(ix,3)],'color','red');
    %}
    %%
    %compute distances to pinhole for these allR reflected rays
    %knownCamPos = matfile(P.camPos).camPos;%
    knownCamPos = matfile([expdir 'camPos.mat']).camPos; 
    %knownCamPos = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/aug18test/camPos.mat').camPos;
    %knownCamPos = matfile([P.data 'camPos_06_28_2022']).camPos;
    %knownCamPos = matfile([P.data 'camPos_06_28_2022']).camPos;
    allTrueDists = [];
    for ix=1:size(allR,1)
        allTrueDists(ix) = distPointToLine(knownCamPos, allSpecPos(ix,:), allR(ix,:));
    end
    
    %%
    % also get the brightness of those specs
    brightness = [];
    %imageCentroids = imageCentroids(dist<closeEnough,:);
    brightness = interp2(double(im), imageCentroids(:,1), imageCentroids(:,2));
    %% also show the original max image so that we can compare the centroids
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
    % set specs not within 2cm to red
    % set specs 0 to 2cm from green to red
    %for ix=1:size(allTrueDistsNormalized,2)
    %    plot(characterizedCentroidsOnThisImage(ix,1),characterizedCentroidsOnThisImage(ix,2),'+','Color',colors(ix,:));
    %end
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

    %DOING: need to save/return/globalize these light2spec (L) rays for use
    %in re-reflecting in the optimizeMEasurements script I'm writing. will
    %need to just save the mostInliersL version (by adding that down below
    %as a stored part during RANSAC)

    % estimate camera position by minimizing some error function
    %errFun = @(c) errT(c, specPos, R);
    %x0 = [M.GLIT_SIDE/2;M.GLIT_SIDE/2;M.GLIT_SIDE];
    %x0 = [0;0;0];
    %x0 = [M.GLIT_SIDE;M.GLIT_SIDE;2*M.GLIT_SIDE];
    %options = optimset('PlotFcns',@optimplotfval);
    %camPosEst = fminsearch(errFun,x0,options)';
    %disp(camPosEst);
    
    % find a good translation estimate using a RANSAC approach
    rng(314159);
    inlierThreshold = 10; % (mm) a reflected ray is an inlier
                          % with respect to a hypothesized camera position
                          % if it pases within 10 millimeters of that 
                          % camera position
                          % draw rig
    figure;
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
    legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
    % table: (made up coords, doesn't matter, mostly for fun)
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    % shown known ground truth camera position
    %knownCamPos = matfile([P.data 'camPos_06_28_2022']).camPos;
    legendItems(size(legendItems,2)+1) = scatter3(knownCamPos(1),knownCamPos(2),knownCamPos(3),100,'blue','filled','o','DisplayName','True Camera');
    % show light source as a dot
    legendItems(size(legendItems,2)+1) = scatter3(lightPos(1),lightPos(2),lightPos(3),'filled','DisplayName','Light');
    %draw all lines red to start
    Rlong = R .* 1000;
    for ix=1:size(Rlong,1)
        % draw reflected rays
        x = [specPos(ix,1,1) specPos(ix,1,1)+Rlong(ix,1,1)]';
        y = [specPos(ix,1,2) specPos(ix,1,2)+Rlong(ix,1,2)]';
        z = [specPos(ix,1,3) specPos(ix,1,3)+Rlong(ix,1,3)]';
        color = [1 0 0];
        %line(x,y,z,'Color',color);
    end
    numInliers = [];
    legPos = size(legendItems,2)+1;
    camroll(-80);
    mostInliersSpecPos = [];
    mostInliersR = [];
    mostInliersL = [];
    for counter=1:10000 %this is constant right now but could (should) be more dynamic/reactive than that
        
        % hypothesize a possible pair of inliers
        idxsRandomTwo = randi(size(R,1),1,2);
        %k = 1; 
        %when hypothesizing, just take the neareast neighbor specs
        %{
        if nearestDistLines(specPos(idxsRandomTwo,1,:), R(idxsRandomTwo,1,:)) > 2 * inlierThreshold
            continue
        end
        %}
        inlierPairFound = false;
        inlierPairK = -1;
        for k=1:size(specPos,2)
            if nearestDistLines(specPos(idxsRandomTwo,k,:), R(idxsRandomTwo,k,:)) < 2 * inlierThreshold
                inlierPairFound = true;
                inlierPairK = k;
                break;
            end
        end
        if ~inlierPairFound
            continue
        end
        % TODO try k=1....K 
        % find the corresponding candidate camera position
        points = specPos(idxsRandomTwo,inlierPairK,:);
        directions = R(idxsRandomTwo,inlierPairK,:);
        candidate = pointBetweenLines(points, directions);
        % sometimes the lines cross near each other behind the glitter plane...
        % but also a position within a few cm in front of the plane is not
        % cool... call it 250mm=25cm
        if candidate(3) < 250
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
        for ix=1:size(inlierIdxs)
            inliersImageSpecPos(ix,:) = imageCentroids(inlierIdxs(ix),:);
            inliersSpecPos(ix,:) = specPos(inlierIdxs(ix),kmin(ix),:);
            inliersR(ix,:) = R(inlierIdxs(ix),kmin(ix),:);
            inliersL(ix,:) = L(inlierIdxs(ix),kmin(ix),:);
            inliersMaxBrightness(ix) = maxBrightness(inlierIdxs(ix),kmin(ix));
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
        end
        %{ one of the big plots
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
        %}
    end
    %legend(legendItems);
    figure;
    histogram(numInliers);
    
    disp(['found ' num2str(max(numInliers)) ' consistent inliers among ' int2str(size(imageCentroids,1)) ' detected sparkles...']);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % now after ransac we can compute for all bright specs the 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% now just for the model with the most inliers, we build up a 
    % camera position estimate
    % estimate camera position by minimizing some error function
    %errFun = @(c) errT(c, mostInliersSpecPos, mostInliersR);
    %x0 = [M.GLIT_SIDE/2;M.GLIT_SIDE/2;M.GLIT_SIDE];
    %x0 = [0;0;0];
    %x0 = [M.GLIT_SIDE;M.GLIT_SIDE;2*M.GLIT_SIDE];
    %options = optimset('PlotFcns',@optimplotfval);
    %camPosEst = fminsearch(errFun,x0,options)';
    %disp(camPosEst);
    camPosEst = nearestPointManyLines(mostInliersSpecPos, mostInliersSpecPos+mostInliersR);
    
    
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
        bestMaxBrightness(ix) = maxBrightness(inlierIdxs(ix),kmin(ix));
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
        trueInlierMaxBrightness(ix) = maxBrightness(trueInlierIdxs(ix),kmin(ix));
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
    
    % draw rig
    figure;
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
    %legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
    % table: (made up coords, doesn't matter, mostly for fun)
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    %legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    % show camera as a dot: (dots=cameras)
    cam=camPosEst;
    legendItems(size(legendItems,2)+1) = scatter3(cam(1),cam(2),cam(3),100,'red','o','filled','DisplayName','Estimated Camera');
    %disp(cam);
    % shown known ground truth camera position
    %knownCamPos = matfile(P.camPos).camera_in_glitter_coords;
    disp('camPosEstimate');
    disp(camPosEst);
    disp('knownCamPos')
    disp(knownCamPos);
    disp('difference (error):');
    disp(norm(cam - knownCamPos));
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
    brightnessNormalized = double(brightness) ./ bestMaxBrightness';% color code for
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
    %for ix=1:size(R,1)
    %    % draw reflected rays
    %    x = [bestSpecPos(ix,1) bestSpecPos(ix,1)+bestR(ix,1)*1000]';
    %    y = [bestSpecPos(ix,2) bestSpecPos(ix,2)+bestR(ix,2)*1000]';
    %    z = [bestSpecPos(ix,3) bestSpecPos(ix,3)+bestR(ix,3)*1000]';
    %    %distance form camera estimate
    %    green = [0 1 0];
    %    red = [1 0 0];
    %    %c = min(1, distNormalized(ix)); %color code by spec closeness
    %    c = 1.0*min(1, brightnessNormalized(ix)); %color code by spec brightness
    %    %disp(c);
    %    color = green + c*(red-green);
    %    %color = [1 c 1];
    %    line(x,y,z,'Color',color);
    %end
    title('green rays are from a brighter sparkle, red from a dimmer one');
    % set viewpoint:
    view([-110 -30]);
    camroll(-80);
    daspect([1 1 1]);
    legend(legendItems);
    
    %
    %%%
    %%%
    %%%%%
    %%%%%%%%
    %%%%%%%%% now plot brightness vs distance to true pinhole
    %%%%%%    
    %%%%%
    %%%
    %
    figure;
    scatter(minTrueDists, brightnessNormalized');
    xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
    ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
    title('Intensity vs Dist to Pinhole: All reflected rays');
    % also do so for just the inliers of the final estimate
    trueDistsInliersOnly = zeros(size(mostInliersR,1),1);
    for ix=1:size(mostInliersR,1)
        trueDistsInliersOnly(ix) = distPointToLine(knownCamPos, mostInliersSpecPos(ix,:), mostInliersR(ix,:));
    end
    %brightnessNormalizedInliersOnly = brightnessNormalized(mostInliersIdxs);
    %brightnessNormalizedInliersOnly = mostInliersMaxBrightness';
    brightnessNormalizedInliersOnly = brightness(mostInliersIdxs) ./ maxBrightness(mostInliersIdxs);
    figure;
    scatter(minTrueDistsInliers, brightnessNormalizedInliersOnly);
    xlabel('Distance to pinhole (from reflected ray to true pinhole (millimeters))');
    ylabel('Intensity (of spec centroids, linearly interpolated, normalized)');
    title('Intensity vs Dist to Pinhole: Inliers Only');
    
    
    %brightnessNormalizedInliersOnly = brightnessNormalized(mostInliersIdxs);
    %brightnessNormalizedInliersOnly = mostInliersMaxBrightness';
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