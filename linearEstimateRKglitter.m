% given camera position estimate T, estimate camera rotation matrix R and
% intrinsic matrix K

function rotAndIntrinsics = linearEstimateRKglitter(impath, camPosEst, pin, mostInliersSpecPos, mostInliersImageSpecPos, expdir)
    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    
    % known points in world coordinates
    worldSpecs = mostInliersSpecPos; % the characterized, canonical spec positions that correspond to the sparkling specs in the image
    imageSpecs = mostInliersImageSpecPos; % the image coordinates of where we find those specs in this image

    % also, worldFiducials and imageFiducials give the fiducial marker point
    % correspondences
    worldFiducials = getFiducialMarkerPts();
    worldFiducials = [worldFiducials zeros(size(worldFiducials,1),1)];

    worldPoints = [worldSpecs; worldFiducials];
    imagePoints = [imageSpecs; pin];

    T = reshape(camPosEst,3,1);
    
    % find R and K by solving linear system
    Q = worldPoints' - T;
    p = imagePoints';
    % build matrix
    A = zeros(2*size(p,2), 8);
    b = zeros(1, 2*size(p,2));
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
    x = A \ b';
    M = [x(1) x(2) x(3); x(4) x(5) x(6); x(7) x(8) 1];

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

    % so R (upper triangular) is scaled K and Q (orthogonal) is the rotation
    K = R ./ R(3,3);
    R = Q;

    % force the main entries of K to be positive, update R accordingly
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

    % sometimes the R we want is negative of the R we find... make sure we
    % get the right one by flipping all 3 axes if needed:
    ztester = [0 0 1]';
    neg = -R*ztester;
    if neg(3) < 0
        R = -R;
    end

    %%
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;    
    title('AFTER DECOMPOSITION: the original image specs (green) and projected by M specs (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = K*R * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end

    %% show the reprojection according to the ground truth/checkerboard
    % results
    camRot = matfile([expdir 'camRot.mat']).camRot;
    camPos = matfile([expdir 'camPos.mat']).camPos;
    checkerParams = matfile([expdir 'camParams.mat']).camParams;
    checkerK = checkerParams.Intrinsics.IntrinsicMatrix';
    figure;
    %plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;    
    % undistort image spec locations
    imageSpecsUndistorted = undistortPoints(imageSpecs, checkerParams);
    plot(imageSpecsUndistorted(:,1),imageSpecsUndistorted(:,2),'gx');hold on;    
    title('checkerboard reprojection: image positions (green) and projected positions (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = checkerK*camRot * (worldSpecs(ix,:)' - camPos');
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end

    %% draw the scene with camera and its frustum
    M = matfile(P.measurements).M;
    figure;
    hold on;
    % draw frustum
    frustumImagePoints = [0 0; 0 M.YRES; M.XRES M.YRES; M.XRES 0];
    % since p = KR(P-T) for world point P to image points p,
    % we get that P = T+(KR)^-1(p)
    frustumWorldPoints = [];
    for ix=1:size(frustumImagePoints,1)
        frustumWorldPoints(ix,:) = inv(K*R) * ([frustumImagePoints(ix,:)';1].*1000) + T;
    end
    for ix=1:size(frustumWorldPoints,1)
        plot3([T(1) frustumWorldPoints(ix,1)],...
              [T(2) frustumWorldPoints(ix,2)],...
              [T(3) frustumWorldPoints(ix,3)],...
              'Color', 'cyan');
    end
    % glitter square:
    gx = [0 M.GLIT_WIDTH M.GLIT_WIDTH 0];
    gy = [0 0 M.GLIT_HEIGHT M.GLIT_HEIGHT];
    gz = [0 0 0 0];
    gc = ['b'];
    patch(gx,gy,gz,gc,'DisplayName', 'Glitter');hold on;
    % monitor:
    mx = [-M.GLIT_TO_MON_EDGES_X -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X+M.MON_WIDTH_MM -M.GLIT_TO_MON_EDGES_X]; 
    my = [-M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y+M.MON_HEIGHT_MM -M.GLIT_TO_MON_EDGES_Y -M.GLIT_TO_MON_EDGES_Y]; 
    mz = [M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES M.GLIT_TO_MON_PLANES]; 
    mc = [.2 .2 .2];
    patch(mx,my,mz,mc,'DisplayName','Monitor');
    % table: (made up coords, doesn't matter, mostly for fun)
    tx = [-400 -400 600 600];
    ty = [-120 -120 -120 -120];
    tz = [-250 1000 1000 -250];
    tc = ['k'];
    patch(tx,ty,tz,tc,'DisplayName','Table');
    axis vis3d;
    axis equal;

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
