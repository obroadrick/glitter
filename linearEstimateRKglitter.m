% estimate rotation and focal length
% this is a very simple version of camera calibration
% in which a camera point p in homogenous coordinates
% for a world point P in world coordinates is given by
% p = K(PR+T) where T is the translation we already solved for
% and R is the rotation from world to camera coordinates we solve for here
% and K is the camera instrinsics matrix which we solve for here as well.
% we parameterize R by 3 components, rodrigues parameters
% we parameterize K by just a single component f so that we have 
% K = [f 0 w/2; 0 f h/2; 0 0 1]; where w and h are the width 
% and height in pixels of the image

function rotAndIntrinsics = linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos, expdir)
    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    MEAS = matfile(P.measurements).M;
    
    % read in image
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
    im = rgb2gray(imread(impath));
    
    %% find spec centroids in image
    %pin = [1217.34838867 5145.87841797; 1005.55084  295.4278; 6501.5874  490.0575; 6501.952 5363.594];
    %pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y
    %pin = [1642.2677 5380.783; 1337.9928 733.52966; 6572.239 726.0792; 6226.173 5270.477];
    M=MEAS;
    tform = getTransform(P, pin);
    imageCentroids = singleImageFindSpecs(im);
    out = transformPointsForward(tform, [imageCentroids(:,1) imageCentroids(:,2)]);
    canonicalCentroids = [out(:,1) out(:,2) zeros(size(out,1),1)];
    pout = [0 0; 0 M.GLIT_HEIGHT; M.GLIT_WIDTH M.GLIT_HEIGHT; M.GLIT_WIDTH 0];%x,y pairs...this is true to glitter coords for the project
    markeradjustments = [M.FIDUCIAL_MARKER_TO_EDGE M.FIDUCIAL_MARKER_TO_EDGE;...
                        M.FIDUCIAL_MARKER_TO_EDGE (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE);...
                        (-M.FIDUCIAL_MARKER_TO_EDGE-M.FIDUCIAL_MARKER_SIZE) M.FIDUCIAL_MARKER_TO_EDGE];
    pout = pout + markeradjustments;
    worldFiducials = [pout zeros(size(pout,1),1)];
    imageFiducials = pin;
    
    %{
    %% match canonical centroids to those in the characterization
    knownCanonicalCentroids = matfile(P.canonicalCentroids).canonicalCentroids;
    K = 10;
    [idx, dist] = knnsearch(knownCanonicalCentroids, canonicalCentroids,...
                                 'K', K, 'Distance', 'euclidean');
    % only consider specs whose match is within .xx millimeters
    %closeEnough = .3;
    %specIdxs = idx(dist<closeEnough);
    %specPos = knownCanonicalCentroids(idx,:);
    specPos = zeros(size(idx,1),K,3);
    for ix=1:size(idx,1)
        specPos(ix,:,:) = knownCanonicalCentroids(idx(ix,:),:);
    end
    % draw vector map where each vector goes from a spec to its nearest spec
    % neighbor
    %figure;
    %quiver(canonicalCentroids(:,1), canonicalCentroids(:,2), knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1), knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2),'LineWidth',2);
    %x = knownCanonicalCentroids(idx(:,1),1)-canonicalCentroids(:,1);
    %y = knownCanonicalCentroids(idx(:,1),2)-canonicalCentroids(:,2);
    %figure;
    %histogram(atan2(y, x));title('histogram of vector directions');
    %xlabel('direction in radians');
    
    %% get spec normals and brightness
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
    %}
    
    %% now make an estimate of the rotation and instrinsics matrices for this
    % camera calibration
    
    % known points in world coordinates
    worldSpecs = mostInliersSpecPos; % the characterized, canonical spec positions that correspond to the sparkling specs in the image
    imageSpecs = mostInliersImageSpecPos; % the image coordinates of where we find those specs in this image
    
    % also, worldFiducials and imageFiducials give the fiducial marker point
    % correspondences
    w = MEAS.XRES;
    h = MEAS.YRES;
    T = reshape(camPosEst,3,1);
    
    % find R and K by solving linear system
    Q = worldSpecs' - T;
    p = imageSpecs';
    % build matrix
    %{
    skewiszero = true;%solve for skew or not
    if skewiszero
        A = [];
        b = [];
        for ix=1:size(p,2)
            q1 = Q(1,ix);
            q2 = Q(2,ix);
            q3 = Q(3,ix);
            px = p(1,ix);
            py = p(2,ix);    
            A(2*ix-1,:) = [-q1 -q3 0 0 0 q1*px q2*px];
            A(2*ix,:) = [0 0 -q1 -q2 -q3 q1*py q2*py];
            b(2*ix-1) = -q3*px;
            b(2*ix) = -q3*py;
        end
        %solve
        x1 = lsqr(A,b');
        x2 = lsqminnorm(A,b');
        x = A \ b';
        M = [x(1) 0 x(2); x(3) x(4) x(5); x(6) x(7) 1];
    else
    %}
    A = [];
    b = [];
    for ix=1:size(p,2)
        q1 = Q(1,ix);
        q2 = Q(2,ix);
        q3 = Q(3,ix);
        px = p(1,ix);
        py = p(2,ix);    
        A(2*ix-1,:) = [-q1 -q2 -q3 0 0 0 q1*px q2*px];
        A(2*ix,:) = [0 0 0 -q1 -q2 -q3 q1*py q2*py];
        b(2*ix-1) = -q3*px;
        b(2*ix) = -q3*py;
    end
    %solve
    %x1 = lsqr(A,b');
    %x2 = lsqminnorm(A,b');
    x = A \ b';
    M = [x(1) x(2) x(3); x(4) x(5) x(6); x(7) x(8) 1];
    Mmatrix = M;
    %% show (before decomposition) the reprojected points to confirm that they make sense
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;
    title('BEFORE DECOMPOSITION: the original image specs (green) and projected by M specs (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = M * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end
    
    %%
    % getting RQ decomposition using matlab's QR decomp (doesn't have RQ)
    [R,Q] = rq(M);
    %{
    disp('should be the same');
    disp(M);
    disp(R*Q);
    %}
    % so R (upper triangular) is scaled K and Q (orthogonal) is the rotation
    K = R ./ R(3,3);
    R = Q;
    Kbeforefix = K;
    Rbeforefix = R;% ok so the determinant of this is also improperly -1
    % there is a symmetry where we sometimes get an R matrix that points
    % the frustum in the exact opposite of the correct direction and so we
    % need to correct for that... TODO is to detect it to automatically
    % correct for it
    %R = rotx(180) * roty(180) * rotz(180) * R;
    %{
    disp('K');
    disp(K);
    disp('R');
    disp(R);
    %}
    %problem and solution alert! woohoo maybe i just need positive
    % entries in k and so i multiply by the right identity matrix (
    % with negatives ones and ones) on the right of k and left of r
    % and then since it is its own inverse i have changed nothin :)?
    %shitty implementation to make sure it is doing what i want at first
    Icorrection = [1 0 0; 0 1 0; 0 0 1];
    if K(1,1) < 0
        Icorrection(1,1) = -1;
    end
    if K(2,2) < 0
        Icorrection(2,2) = -1;
    end
    if K(3,3) < 0
        Icorrection(3,3) = -1;
    end
    K = K * Icorrection;
    R = Icorrection * R;

    % and then just make it negative. because.
    %R = -R;
    % well actually check whether we got the negative of the desired
    % solution or not.
    % the wrong solution (-R_desired) will have rotated the z axis from
    % pointing into the glitter plane to pointing the other way and so we
    % can just choose whichever matrix rotates the z axis by more
    % we know that the z-axis of the canonical glitter coordinates points
    % out towards the camera and the desired camera z-axis points back
    % towards the glitter plane, and so the z-entry of the rotate z-tester
    % vector (canonical z basis vector) should be negative.
    ztester = [0 0 1]';
    neg = -R*ztester;
    if neg(3) < 0
        R = -R;
    end

    %{
    if det(R) < 0
        disp('det(R) < 0 rn');
        
        % so if we want a "proper" rotation matrix, we need to make one of
        % the K entries negative... easy enough:
        Iproper = [1 0 0; 0 -1 0; 0 0 1];
        K = K * Iproper;
        R = Iproper * R;

        disp('now det(R) = ');
        disp(det(R));
    end
    %}

    %R = rotx(180) * roty(180) * rotz(180) * R;
    %{
    disp('K');
    disp(K);
    disp('R');
    disp(R);
    %}
    %{
    % weird thing.. an improper rotation matrix (one which rotates and reflects (flips an axis))
    % needs to be avoided (at the very least because of some complaints
    % from matlab functions but perhaps also for some slightly deeper
    % reason involving finding a desirable form of the solution)

    % attempt at fixing: just flip the z axis. gets us back to the desired
    % coordinate system and it's really unclear that we care which way the
    % z axis is going... so if it's improper, just flip it...
    if det(R) < 0
        disp('hereeeee');
        R = R*[-1 0 0; 0 1 0; 0 0 1;];
    end
    % this doesn't work since it just plainly changes the rotation and so
    the results are wrong by a reflection

    % the next thing to try is just flipping an axis of the data being fit
    to... why this would need to happen sometimes is not clear to me now,
    but I will try it 
    %}

    %{
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% if we get determinant -1 in the orthogonal matrix, we need to re-compute
    % the solution matrix but with one of the axes of the input points
    % being flipped (to get a results with a proper rotation matrix 

    if det(R) < 0
        disp('determinant -1, re-running on augmented inputs');
    end
    % known points in world coordinates
    worldSpecs = mostInliersSpecPos; % the characterized, canonical spec positions that correspond to the sparkling specs in the image
    imageSpecs = mostInliersImageSpecPos; % the image coordinates of where we find those specs in this image
    
    % NEW HERE FOR THIS CASE
    % we can't flip two axes (because that is the same set of points up to
    % a proper rotation. so instead we need to flip an odd number of axes, 
    % which we achieve by flipping a single axis.) shouldn't matter from
    % which set we do this, but ill choose a natural-feeling choice which
    % is the z-axis of the world coordinate system

    % ok doing that made me realize this issue could be related to the fact
    % that the worldSpecs are positioned at 0... which leaves some
    % ambiguity about the direction of the z-axis (making it feel to me
    % like there are two symmetrical solutions that exist, but of course a
    % linear system should either have 1 or infinitely many solutions, and
    % so i'm still confused, but anyways going to try now having a neglible
    % but nonzero z-axis entry for the worldspecs and play with its sign to
    % see if this fixes the problem)

    % after that testing the strange behavior now is that either negative
    % or positive entries for the third axis don't affect the orthogonal
    % matrix's determinant (it is still -1 in both cases) which is odd and
    % unfortunate... odd because -1 det means a reflection (improper
    % rotation where rotation and reflection occurs) and so reflecting one
    % of the axes should get us into the other version of the axes and so
    % the reflection should not be needed anymore (and there only two
    % possibilities for the handed-ness of the axes)

    % that seems plainly wrong to me (and less fuzzy but more likely jsut
    % wrong so now trying to see if it is a consequence of a the precision
    % and the solutions that do or don't have reflection are really close
    % together which makes sense since z axis entries are near zero
    for ix=1:size(worldSpecs)
        worldSpecs(ix,3) = 0;
    end
    %{
    for ix=1:size(imageSpecs)
        imageSpecs(ix,1) = imageSpecs(ix,1) * -1;
    end
    %}


    % also, worldFiducials and imageFiducials give the fiducial marker point
    % correspondences
    w = MEAS.XRES;
    h = MEAS.YRES;
    T = reshape(camPosEst,3,1);
    
    % find R and K by solving linear system
    Q = worldSpecs' - T;
    p = imageSpecs';
    % build matrix
    %{
    skewiszero = true;%solve for skew or not
    if skewiszero
        A = [];
        b = [];
        for ix=1:size(p,2)
            q1 = Q(1,ix);
            q2 = Q(2,ix);
            q3 = Q(3,ix);
            px = p(1,ix);
            py = p(2,ix);    
            A(2*ix-1,:) = [-q1 -q3 0 0 0 q1*px q2*px];
            A(2*ix,:) = [0 0 -q1 -q2 -q3 q1*py q2*py];
            b(2*ix-1) = -q3*px;
            b(2*ix) = -q3*py;
        end
        %solve
        x1 = lsqr(A,b');
        x2 = lsqminnorm(A,b');
        x = A \ b';
        M = [x(1) 0 x(2); x(3) x(4) x(5); x(6) x(7) 1];
    else
    %}
    A = [];
    b = [];
    for ix=1:size(p,2)
        q1 = Q(1,ix);
        q2 = Q(2,ix);
        q3 = Q(3,ix);
        px = p(1,ix);
        py = p(2,ix);    
        A(2*ix-1,:) = [-q1 -q2 -q3 0 0 0 q1*px q2*px];
        A(2*ix,:) = [0 0 0 -q1 -q2 -q3 q1*py q2*py];
        b(2*ix-1) = -q3*px;
        b(2*ix) = -q3*py;
    end
    %solve
    %x1 = lsqr(A,b');
    %x2 = lsqminnorm(A,b');
    x = A \ b';
    M = [x(1) x(2) x(3); x(4) x(5) x(6); x(7) x(8) 1];
    Mmatrix = M;
    %% show (before decomposition) the reprojected points to confirm that they make sense
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;
    title('BEFORE DECOMPOSITION: the original image specs (green) and projected by M specs (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = M * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end
    
    %%
    % getting RQ decomposition using matlab's QR decomp (doesn't have RQ)
    [R,Q] = rq(M);
    %{
    disp('should be the same');
    disp(M);
    disp(R*Q);
    %}
    % so R (upper triangular) is scaled K and Q (orthogonal) is the rotation
    K = R ./ R(3,3);
    R = Q;
    Kbeforefix = K;
    Rbeforefix = R;% ok so the determinant of this is also improperly -1
    % there is a symmetry where we sometimes get an R matrix that points
    % the frustum in the exact opposite of the correct direction and so we
    % need to correct for that... TODO is to detect it to automatically
    % correct for it
    %R = rotx(180) * roty(180) * rotz(180) * R;
    %{
    disp('K');
    disp(K);
    disp('R');
    disp(R);
    %}
    %problem and solution alert! woohoo maybe i just need positive
    % entries in k and so i multiply by the right identity matrix (
    % with negatives ones and ones) on the right of k and left of r
    % and then since it is its own inverse i have changed nothin :)?
    %shitty implementation to make sure it is doing what i want at first
    Icorrection = [1 0 0; 0 1 0; 0 0 1];
    if K(1,1) < 0
        Icorrection(1,1) = -1;
    end
    if K(2,2) < 0
        Icorrection(2,2) = -1;
    end
    if K(3,3) < 0
        Icorrection(3,3) = -1;
    end
    K = K * Icorrection;
    R = Icorrection * R;
    %R = rotx(180) * roty(180) * rotz(180) * R;
    %{
    disp('K');
    disp(K);
    disp('R');
    disp(R);
    %}
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %}
    %%
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;    
    title('AFTER DECOMPOSITION: the original image specs (green) and projected by M specs (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = K*R * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end

    %% compare to expected rotation matrix by matlab checkerboard calibration
    %camRot = matfile([P.camRot]).camRot;
    camRot = matfile([expdir 'camRot.mat']).camRot;

    % get rorigues parameters from rotation matrix
    ourRodrig = undoRodrigues(R);
    matlabRodrig = undoRodrigues(camRot);
    %{
    disp('Rodrigues parameters as estimated by our glitter');
    disp(ourRodrig)
    disp('Rodrigues parameters as estimated by Matlab checkerboard calibration');
    disp(matlabRodrig);
    %}
    %%
    % show the reprojection according to the ground truth/checkerboard
    % results
    checkerParams = matfile([expdir 'camParams.mat']).camParams;
    checkerK = checkerParams.Intrinsics.IntrinsicMatrix;
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;    
    title('checkerboard projection: image positions (green) and projected positions (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = K*camRot * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end

    %% we can also compare the rotations directly by measuring the angle or
    % rotation from one to the other (which gives a notion of how far one would
    % have to rotate to correct for the error)...
    % rotate the same vector by both rotation matrices
    v = [1;0;0];
    ourRv = R*v;
    matlabRv = camRot*v;
    diffR = R * camRot';
    alsotheta = acos((trace(diffR) - 1) / 2);%how to find angle of rotation
    theta = norm(undoRodrigues(diffR));
    %disp(alsoshouldbetheta);%useful debugging to confirm angle computation
    thetadegs = theta * 180 / pi;
    %disp(theta);
    %{
    disp('the difference between these two rotation matrices in degrees:');
    disp(thetadegs);
    %}
    

    %[Q,R] = qr(M);
    
    % way of doing it with search:
    %             errRK(fx,   fy,   s, w, h, r1,   r2,   r3,   p, Pts, T)
    %{
    plottingFigure = figure;
    errFun = @(x) errRK(x(1), x(2), x(3), w, h, x(4), x(5), x(6), imageSpecs,...
        worldSpecs, imageFiducials, worldFiducials, T, plottingFigure);
    %x0 = [10^(-3)*12000 10^(-3)*12000 0 3 -1.5 -1.5]';% old starting guess
    % use the results of the linear system solution to give us the starting
    % guess for the minimization
    x0 = [10^(-3)*K(1,1) 10^(-3)*K(2,2) K(1,2) ourRodrig(1) ourRodrig(2) ourRodrig(3)];
    % use the camera known points for x0 TODO
    options = optimset('PlotFcns',@optimplotfval);
    xf = fminsearch(errFun, x0, options);
    fx = xf(1);
    fy = xf(2);
    s = xf(3);
    r1 = xf(4);
    r2 = xf(5);
    r3 = xf(6);
    R = rodrigues(r1,r2,r3);
    %
    s = 0;
    K = [10^(3)*fx s w/2; 0 10^(3)*fy h/2; 0 0 1];
    disp('and here are the optimized K and R respectively matrices from using initial guess from linear system:');
    disp(K);
    disp(R);
    %}
    %% draw the scene with camera and its frustrum
    %R=R';
    M = matfile(P.measurements).M;
    figure;
    %{
    disp(R);
    disp(T);
    %}
    %pose = rigid3d(R',T');
    hold on;
    %camObj = plotCamera('AbsolutePose',pose,'Opacity',0,'Size',35);
    % draw frustum

    frustumImagePoints = [0 0; 0 M.YRES; M.XRES M.YRES; M.XRES 0];
    % since p = KR(P-T) for world point P to image points p,
    % we get that P = T+(KR)^-1(p)
    frustumWorldPoints = [];
    frustumWorldPointsCheck = [];
    for ix=1:size(frustumImagePoints,1)
        %disp([frustumImagePoints(ix,:)';1]);
        frustumWorldPoints(ix,:) = inv(K*R) * ([frustumImagePoints(ix,:)';1].*1000) + T;
        frustumWorldPointsCheck(ix,:) = inv(Mmatrix) * ([frustumImagePoints(ix,:)';1].*1000) + T;
    end
    for ix=1:size(frustumWorldPoints,1)
        plot3([T(1) frustumWorldPoints(ix,1)],...
              [T(2) frustumWorldPoints(ix,2)],...
              [T(3) frustumWorldPoints(ix,3)],...
              'Color', 'cyan');
    end
    %%%%%%%%
    axis vis3d;
    
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
    mc = [.2 .2 .2];
    legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
    % table: (made up coords, doesn't matter, mostly for fun)
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure; hold on;
    for ix=1:size(frustumWorldPoints,1)
        plot3([T(1) frustumWorldPointsCheck(ix,1)],...
              [T(2) frustumWorldPointsCheck(ix,2)],...
              [T(3) frustumWorldPointsCheck(ix,3)],...
              'Color', 'cyan');
    end
    %disp(frustumWorldPoints);
    %disp(frustumWorldPointsCheck);
    axis vis3d;
    
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
    mc = [.2 .2 .2];
    legendItems(size(legendItems,2)+1) = patch(mx,my,mz,mc,'DisplayName','Monitor');
    % table: (made up coords, doesn't matter, mostly for fun)
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    legendItems(size(legendItems,2)+1) = patch(tx,ty,tz,tc,'DisplayName','Table');

    % visualize the predicted transformation being applied to the canonical
    % axes
    visAxesRT(R,T);

    % returns
    omega = undoRodrigues(R);
    fx = K(1, 1);
    fy = K(2,2);
    cx = K(1,3);
    cy = K(2,3);
    s = K(1,2);
    rotAndIntrinsics = [omega fx fy cx cy s];
end
