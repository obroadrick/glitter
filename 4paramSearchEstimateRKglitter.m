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

function rotAndIntrinsics = 4paramSearchEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos)
    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    MEAS = matfile(P.measurements).M;
    
    % read in image
    %impath = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/2022-06-10T18,17,57circle-calib-W1127-H574-S48.jpg';
    im = rgb2gray(imread(impath));
    
    %% find spec centroids in image
    %pin = [1217.34838867 5145.87841797; 1005.55084  295.4278; 6501.5874  490.0575; 6501.952 5363.594];
    %pin = [ 1118, 5380; 596, 415; 6365, 393; 6065, 5402];% x,y
    %pin = [1642.2677 5380.783; 1337.9928 733.52966; 6572.239 726.0792; 6226.173 5270.477];
    M = MEAS;
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

    
    %% now make an estimate of the rotation and instrinsics matrices for this
    % camera calibration
    
    % known points in world coordinates
    worldSpecs = mostInliersSpecPos; % the characterized, canonical spec positions that correspond to the sparkling specs in the image
    imageSpecs = mostInliersImageSpecPos; % the image coordinates of where we find those specs in this image
    figure;
    tiledlayout(1,2);
    nexttile;
    title('world specs');
    plot(worldSpecs(:,1),worldSpecs(:,2));
    nexttile;
    title('image specs');
    plot(imageSpecs(:,1),imageSpecs(:,2));
    % also, worldFiducials and imageFiducials give the fiducial marker point
    % correspondences
    w = MEAS.XRES;
    h = MEAS.YRES;
    T = reshape(camPosEst,3,1);
    
    Q = worldSpecs' - T;
    p = imageSpecs';

    % search over the 8 parameters (4 points at corners) that are the
    % fiducial markers that make the best homography M that can be
    % decomposed into K and R

    plottingFigure = figure;
    errFun = @(x) 4cornerErr(x, plottingFigure);
    %errFun = @(x) errRK(x(1), x(2), x(3), w, h, x(4), x(5), x(6), imageSpecs,...
    %    worldSpecs, imageFiducials, worldFiducials, T, plottingFigure);
    
    % start with a guess that says that the four corners 

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
    

    [R,Q] = rq(M);


    %% compare to expected rotation matrix by matlab checkerboard calibration
    camRot = matfile([P.camRot]).camRot;
    % get rorigues parameters from rotation matrix
    ourRodrig = undoRodrigues(R);
    matlabRodrig = undoRodrigues(camRot);
    %{
    disp('Rodrigues parameters as estimated by our glitter');
    disp(ourRodrig)
    disp('Rodrigues parameters as estimated by Matlab checkerboard calibration');
    disp(matlabRodrig);
    %}

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
    
    %% now showing the same thing (reprojection) that we showed before
    % SHOULD give the same result... we hope
    % show (before decomposition) the reprojected points to confirm that they make sense
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;    
    title('the original image specs (green) and projected by M specs (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = K*R * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end
    
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
    %pose = rigid3d(R,T');
    hold on;
    %camObj = plotCamera('AbsolutePose',pose,'Opacity',0,'Size',35);
    % draw frustum



    frustumImagePoints = [0 0; 0 M.YRES; M.XRES M.YRES; M.XRES 0];
    % since p = KR(P-T) for world point P to image points p,
    % we get that P = T+(KR)^-1(p)
    frustumWorldPoints = [];
    for ix=1:size(frustumImagePoints,1)
        frustumWorldPoints(ix,:) = T + 1000 * inv(K*R) * [frustumImagePoints(ix,:)';1];
    end
    for ix=1:size(frustumWorldPoints,1)
        plot3([T(1) -frustumWorldPoints(ix,1)],...
              [T(2) -frustumWorldPoints(ix,2)],...
              [T(3) -frustumWorldPoints(ix,3)],...
              'Color', 'cyan');
    end
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


    % returns
    omega = undoRodrigues(R);
    fx = K(1, 1);
    fy = K(2,2);
    cx = K(1,3);
    cy = K(2,3);
    s = K(1,2);
    rotAndIntrinsics = [omega fx fy cx cy s];
end
