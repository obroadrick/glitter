% undistorts an nx2 array of image points using the radial distortion model
% and using the radial distortion coefficients in the vector k (assumed to
% be of length >= 2)
function newPin = undistortPin(pin, k)
    pin = transform2normalizedImageCoords(pin);
    % compute new points
    newPin = [];
    for ix=1:size(pin,1)
        x = pin(ix,1);
        y = pin(ix,2);
        r2 = x^2 + y^2;
        newPin(ix,1) = x*(1 + k(1)*r2 + k(2)*r2^2);
        newPin(ix,2) = y*(1 + k(1)*r2 + k(2)*r2^2);
    end
    % transform the new points back into image coordinates
    newPin = transformBack(newPin);
end

% assumes that the measurements file at 
% /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/measurements.mat
% contains the relevant image dimensions
function pts = transform2normalizedImageCoords(pin)
    pts = [];
    M = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/measurements.mat').M;
    % transform points to normalized image coordinates:
    for ix=1:size(pin,1)
        pts(ix,1) = (pin(ix,1) - (M.XRES / 2)) / (M.XRES / 2);
        pts(ix,2) = (pin(ix,2) - (M.YRES / 2)) / (M.YRES / 2);
    end
end
function pts = transformBack(pin)
    pts = [];
    M = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/measurements.mat').M;
    % transform points to normalized image coordinates:
    for ix=1:size(pin,1)
        pts(ix,1) = (pin(ix,1) * (M.XRES / 2)) + (M.XRES / 2);
        pts(ix,2) = (pin(ix,2) * (M.YRES / 2)) + (M.YRES / 2);
    end
end
