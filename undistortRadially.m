function PtsU = undistortRadially(Pts, k1, k2)
    % for radial distortion coefficients k1, k2, undistort
    % the passed points Pts and return the undistorted points p
    P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
    M = matfile(P.measurements).M;
    res = [double(M.XRES) double(M.YRES)];
    
    Pts = double(Pts);
    disp(Pts);
    % get porig from image coordinates into centered, normalized coordinates
    Pts(:,:) = (Pts(:,:) - (res/2.0)) ./ (res/2.0);
    %Pts(:,:) = (Pts(:,:)./ (res/2.0)) - res/2.0 ;
    disp(Pts);

    % undistort according to p' = p(1+k1r^2 +k2r^4)
    for ix=1:size(Pts,1)
        r2 = Pts(ix,1)^2 + Pts(ix,2)^2;
        PtsU(ix,:) = Pts(ix,:) * (1 + k1*r2 + k2*r2^2);
    end
    disp(PtsU);
    % get back into image coordinates
    PtsU(:,:) = PtsU(:,:) .* (res/2.0) + (res/2.0);    
    %PtsU(:,:) = (PtsU(:,:) + res/2.0) .* double(res/2.0);    
    disp(PtsU);

end
