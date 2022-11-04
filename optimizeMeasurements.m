% a function that takes a set of measurements as input and produces new
% surface normals for a given characterization and performs a glitter
% camera calibration for the new surface normals

%% INPUTS
% the input characterization
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep18characterization(new-1)/';
%charDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep19characterization(new-2)/';
setPaths(chardir);
% the input camera calibration test case to use to test new surface normals
expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/oct25_nikonz7_35mm/';


%% FUNCTIONALITY

%now suppose that at this point we have been given a new set of
%measurements, charM, and we want to see how re-characterizing the glitter
%sheet with this set of measurements M affects the camera position estimate
%error. so we need to (1) re-compute the spec normals and then (2)
%re-calibrate according to those normals.
% as an initial measurement set charM we take whatever is already there:
charM = matfile([chardir 'measurements.mat']).M;

% 1. recompute the normals based on the given measurements
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
newNormalsPath = computeNormals(P, chardir, charM);
newNormals = matfile(newNormalsPath).specNormals;

% 2. do a sparkle calibration using these normals
% get inputs (image, fiducial marker points, ambient image, light position)
pointLightImageIndex = 1;
impath = [expdir int2str(pointLightImageIndex) 'single.JPG'];
ambientImage = rgb2gray(imread([expdir 'blank1.JPG']));
pin = getPoints(expdir);
w = 3840;h = 2160;xoff = 500;yoff = 300;r = 9;
positions = [xoff yoff; w-xoff h-yoff; xoff h-yoff; w-xoff yoff];
for ix=1:size(positions,1)
    positions(ix,:) = positions(ix,:) + r/2;
end
lightPos = screenPosToWorldPos(positions(pointLightImageIndex,:), matfile([expdir 'measurements.mat']).M);

% estimate translation
warning('off','MATLAB:singularMatrix'); set(0,'DefaultFigureVisible','off');
[camPosEst, mostInliersSpecPos, mostInliersImageSpecPos, other] = estimateTglitter(impath, lightPos, pin, expdir, ambientImage);
%%
mostInliersL = other.mostInliersL; % this extra return of the estimateTglitter function gives us the rays from light position to spec locations
mostInliersSpecPos = other.mostInliersSpecPos;
mostInliersIdxs = other.mostInliersIdxs;
lightPos = other.lightPos;
mostInliersRoriginal = other.mostInliersR;
overallIdx = other.overallIdx;
mostInliersKmin = other.mostInliersKmin;

%temporary debugging code: I know there is an error since this should give
% the same result twice and so i am going to debug by visualizing where the
% returned values from esetimateTglitter fall and then if they are right
% hopefully i will visualize where the reflected rays i compute down in (a)
% fall and then finally make sure that nearestPointManyLines isn't the
% issue if we make it htat far...
%first: visualize outputs of estimateTglitter
%{
figure;
hold on;
for ix=1:size(mostInliersSpecPos,1)
    line([mostInliersSpecPos(ix,1) mostInliersSpecPos(ix,1)+mostInliersL(ix,1)],...
        [mostInliersSpecPos(ix,2) mostInliersSpecPos(ix,2)+mostInliersL(ix,2)],...
        [mostInliersSpecPos(ix,3) mostInliersSpecPos(ix,3)+mostInliersL(ix,3)],...
        'color', 'yellow');
end
%}
figure;
hold on;
for ix=1:size(mostInliersSpecPos,1)
    line([mostInliersSpecPos(ix,1) lightPos(1)],...
        [mostInliersSpecPos(ix,2) lightPos(2)],...
        [mostInliersSpecPos(ix,3) lightPos(3)],...
        'color', 'yellow');
end

% now to compute the error, we just have to (a) reflect the light rays
% given in mostInliersL off the specs according to the new surface normals
% and then (b) compute the new camera position estimate according to these
% rays
% (Note that if we actually did the whole ransac again we may get different
% results but i want to make this optimization feasible in terms of speed
% and so we just use these rays instead.)
%
% (a) reflect the light rays
% given in mostInliersL off the specs according to the new surface normals
%R = reflect(mostInliersL, newNormals(mostInliersIdxs,:));%first try where
mostInliersOverallIdxs = [];
for ix=1:size(mostInliersSpecPos,1)
    mostInliersOverallIdxs(ix) = overallIdx(mostInliersIdxs(ix), mostInliersKmin(ix));
end
mostInliersNewNormals = newNormals(mostInliersOverallIdxs,:);
R = reflect(mostInliersSpecPos, mostInliersNewNormals, lightPos);
R = reshape(R,size(R,1),size(R,3));

% (b) compute the new camera position estimate according to these
% rays
camPosEst = nearestPointManyLines(mostInliersSpecPos, mostInliersSpecPos+R);
knownCamPos = matfile([expdir 'camPos.mat']).camPos; 
disp('camPosEstimate');
disp(camPosEst);
disp('knownCamPos');
disp(knownCamPos);
disp('difference (error):');
disp(norm(camPosEst - knownCamPos));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% functions
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
%{
function R = reflect(specPos, specNormals, lightPos)
    % computes the position of light reflected off the passed spec
    % positions with specNormals from lightPos assuming that specPos is of
    % the shape nxkx3 where n is number of specs, k is number of
    % hypothesized positions for that spec, and 3 is number of dimensions
    % (x,y,z)
    R=[];
    for ix=1:size(specPos,2)
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
end
%}
function R = reflectNew(L, specNormals)
    % computes the position of light reflected off the passed spec
    % positions with specNormals from lightPos assuming that specPos is of
    % the shape nxkx3 where n is number of specs, k is number of
    % hypothesized positions for that spec, and 3 is number of dimensions
    % (x,y,z)
    % compute reflected rays: Ri = Li − 2(Li dot Ni)Ni
    % where Li is normalized vector from spec to light
    %       Ni is normalized normal vector
    %       Ri is normalized reflected vector
    %L = (lightPos - reshape(specPos(:,:),size(specPos,1),...
    %    size(specPos,3))) ./ vecnorm(lightPos - reshape(specPos(:,:),size(specPos,1),size(specPos,3)), 2, 2);
    whos L
    whos specNormals
    R = [];
    for ix=1:size(L,1)
        R(ix,:) = L(ix,:) - 2 * dot(L(ix,:), specNormals(ix,:)) .* specNormals(ix,:);
    end
    %R(:,:) = -1.*R(:,:);
end
function R = reflect(specPos, specNormals, lightPos)
    % computes the position of light reflected off the passed spec
    % positions with specNormals from lightPos assuming that specPos is of
    % the shape nxkx3 where n is number of specs, k is number of
    % hypothesized positions for that spec, and 3 is number of dimensions
    % (x,y,z)
    specPos = reshape(specPos, size(specPos,1), 1, size(specPos,2));
    specNormals = reshape(specNormals, size(specNormals,1), 1, size(specNormals,2));
    R=[];
    for ix=1:size(specPos,2)
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
end
