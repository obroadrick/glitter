% error function for the rotation and intrinsics part of the camera
% calibration

% in which a camera point p in homogenous coordinates
% for a world point Pt in world coordinates is given by
% p = K(PtR+T) where T is the translation we already solved for
% and R is the rotation from world to camera coordinates we solve for here
% and K is the camera instrinsics matrix which we solve for here as well.
% we parameterize R by 3 components, rodrigues parameters
% we parameterize K by just a single component f so that we have 
% K = [f 0 w/2; 0 f h/2; 0 0 1]; where w and h are the width 
% and height in pixels of the image

function e = errRK(f, w, h, r1, r2, r3, p, Pts, T)
    % f is the focal length (assumed same for x and y)
    % w and h are width and height in pixels of the image
    % r1, r2, r2 are the rodrigues parameters of the candidate rotation
    % p is a list of the known image points
    % Pts is a corresponding list of the known world points

    % get intrinsics matrix
    K = [f 0 w/2; 0 f h/2; 0 0 1];

    % get rotation matrix
    R = rodrigues(r1,r2,r3);
    %omega = [r1;r2;r3]; % the vector we parameterized
    %theta = norm(omega); % angle to rotate by (following right hand rule)
    %n = omega ./ theta; % axis of rotation normalized
    %nx = [0 -n(3) n(2); n(3) 0 -n(1); -n(2) n(1) 0]; % cross product as a matrix
    % rodrigues' formula to get rotation matrix from the parameters
    %R = [1 0 0; 0 1 0; 0 0 1] + sin(theta).*nx + (1-cos(theta)).*nx^2;

    % now estimate the image coordinates for all the world points in Pts
    pest = [];
    for ix=1:size(Pts,1)
        %disp(size(K));
        %disp(size(R));
        %disp(size(Pts(ix,:)));
        %disp(size(T));
        Pnt = reshape(Pts(ix,:),3,1);
        pest(ix,:) = K * (R * Pnt + T);
        pest(ix,:) = pest(ix,:) ./ pest(ix,3);
    end

    % compute error as a sum of the square differences
    e = 0;
    for ix=1:size(p,1)
        %disp(size(pest(ix,:)));
        %disp(size(p(ix,:)));
        e = e + norm(pest(ix,1:2) - p(ix, :))^2;
    end
    e = sqrt(e / size(pest,1));
end