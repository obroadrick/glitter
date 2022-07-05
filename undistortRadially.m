function PtsU = undistortRadially(Pts, k1, k2)
    % for radial distortion coefficients k1, k2, undistort
    % the passed points Pts and return the undistorted points p
    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    M = matfile(P.measurements).M;
    res = [double(M.XRES) double(M.YRES)];
    
    Pts = double(Pts);
    %disp(Pts);
    % get porig from image coordinates into centered, normalized coordinates
    Pts(:,:) = (Pts(:,:) - (res/2.0)) ./ (res/2.0);
    %Pts(:,:) = (Pts(:,:)./ (res/2.0)) - res/2.0 ;
    %disp(Pts);

    % undistort by approximating the true point as the distorted point,
    % finding the distortion, then getting a better estimate of the true
    % point, then finding its distortion, and so forth a few times to get
    % an approximately undistorted point.
    % begin assuming PtsU = Pts (undistorted points are just the distorted)
    PtsU = Pts;
    distortion = 0;
    numIters = 5; % same as opencv (5) and it is noted that 2 iterations
    % already is very accurate for little distortion in:
    % https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=609468
    for iters=1:numIters
        for ix=1:size(PtsU,1)
            r2 = PtsU(ix,1)^2 + PtsU(ix,2)^2;
            distortion = PtsU(ix,:) * (k1*r2 + k2*r2^2);
            PtsU(ix,:) = Pts(ix,:) - distortion;
        end
    end
    %disp(PtsU);
    % get back into image coordinates
    PtsU(:,:) = PtsU(:,:) .* (res/2.0) + (res/2.0);    
    %PtsU(:,:) = (PtsU(:,:) + res/2.0) .* double(res/2.0);    
    %disp(PtsU);
end
