%goal of this scipt: optimize over the characterization-time camera
%position to minimize the distance between the average camera position
%according to sparkle-estimates and checker-estimates. then see if these
%normals "generalize" in the sense that they make another camera position's
%test set of sparkle calibrations also agree with the checkerboard
%calibrations (implying that we found the "true" camera position for the
%original characterization)

% the input characterization
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep18characterization(new-1)/';
%charDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep19characterization(new-2)/';
setPaths(chardir);

testcases = loadTestCases();

train = testcases.jan12;
test = testcases.jan13;
testTwo = testcases.feb10;

numcases = 10; 

for index=1:numcases
    input = train(index);

    expdir{index} = input.expdir;
    curExpdir = expdir{index};
    impath{index} = [curExpdir input.impath];
    lightPos{index} = matfile([curExpdir input.lightPosFname]).lightPos;
    %fprintf('\n%s\n',input.name);
    skew = input.skew;
end
%characterization measurements (trusted)
charM = matfile([chardir 'measurements.mat']).M;

%initial charCamPos guess (whatever was originally used, found by checkers)
charCamPos = matfile([chardir 'camPos.mat']).camPos;

% recompute normals
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
newNormalsPath = computeNormals(P, chardir, charM);
newNormals = matfile(newNormalsPath).specNormals;

% get the original lighting means for re-computing normals during the
% optimization
lightingMeans = matfile([chardir 'lightingmeans.mat']).means;

% do only the translation-estimating part of a sparkle calibration
warning('off','MATLAB:singularMatrix');
set(0,'DefaultFigureVisible','off');
other.inlierThreshold = 20;
% pass a setting to get a quick estimate rather than a fully-fine-tuned
% estimate of the camera position
other.quickEstimate = true;
for index=1:numcases
    [camPosEst, ~, mostInliersImageSpecPosCur, other] = estimateTglitter(impath{index}, ...
                                                lightPos{index}, ...
                                                getPoints(expdir{index}), ...
                                                expdir{index}, ...
                                                -1, ...
                                                skew, ...
                                                other);
    mostInliersL{index} = other.mostInliersL; % this extra return of the estimateTglitter function gives us the rays from light position to spec locations
    mostInliersSpecPos{index} = other.mostInliersSpecPos;
    mostInliersIdxs{index} = other.mostInliersIdxs;
    lightPos{index} = other.lightPos;
    mostInliersRoriginal{index} = other.mostInliersR;
    overallIdx{index} = other.overallIdx;
    mostInliersKmin{index} = other.mostInliersKmin;
    mostInliersImageSpecPosAll{index} = mostInliersImageSpecPosCur;
    % get the relevant lighting means
    for ix=1:size(mostInliersSpecPos{index},1)
        mostInliersOverallIdxs{index}(ix) = overallIdx{index}(mostInliersIdxs{index}(ix), mostInliersKmin{index}(ix));
    end
    mostInliersLightingMeans{index}(:,:) = lightingMeans(:,mostInliersOverallIdxs{index});
end

% the optimization begins: for each charCamPos we use
% these light2spec rays, re-compute the surface normals, re-reflect, and
% then use error in the avg camera position estimate that results as the 
% error function (for time, simply using the nearest-point-to-lines
% heuristic to get the sparkle estimate of camera position)
set(0,'DefaultFigureVisible','off');
%{
errFun = @(x) computeError(x(1), x(2), x(3), 664.32, 373.68,...
                        chardir, expdir, mostInliersLightingMeans, ...
                        mostInliersSpecPos, camPos, lightPos);
%}
%% do the "training" optimization
GLIT_TO_MON_PLANES = charM.GLIT_TO_MON_PLANES;
GLIT_TO_MON_EDGES_X = charM.GLIT_TO_MON_EDGES_X;
GLIT_TO_MON_EDGES_Y = charM.GLIT_TO_MON_EDGES_Y;
MON_WIDTH_MM = charM.MON_WIDTH_MM;
MON_HEIGHT_MM = charM.MON_HEIGHT_MM;

%campos and measurements
errFun = @(x) computeError(x(4), x(5), x(6),MON_WIDTH_MM, ...
                              MON_HEIGHT_MM, chardir, expdir, ...
                              mostInliersLightingMeans, ...
                              mostInliersSpecPos, x(1:3), lightPos, ...
                              train, mostInliersImageSpecPosAll, false);
%{
errFun = @(x) computeError(x(1), x(2), x(3),MON_WIDTH_MM, ...
                              MON_HEIGHT_MM, chardir, expdir, ...
                              mostInliersLightingMeans, ...
                              mostInliersSpecPos, charCamPos, lightPos, ...
                              train, mostInliersImageSpecPosAll, false);
%}
x0 = [charCamPos, GLIT_TO_MON_PLANES,...
            GLIT_TO_MON_EDGES_X,...
            GLIT_TO_MON_EDGES_Y];
options = optimset('PlotFcns',@optimplotfval);

xf = fminsearch(errFun, x0, options);

%% get results of the previous optimization to visualize
% compute the error for the "train" set based on the new characterization 
% found by the optimization using the "train" set (to visualize)

%campos and measurements
[error,results] = computeError(xf(4),xf(5),xf(6),MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdir, ...
                              mostInliersLightingMeans, ...
                              mostInliersSpecPos, xf(1:3), lightPos, ...
                              train, mostInliersImageSpecPosAll, true);
%{
[error,results] = computeError(xf(1),xf(2),xf(3),MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdir, ...
                              mostInliersLightingMeans, ...
                              mostInliersSpecPos, charCamPos, lightPos, ...
                              train, mostInliersImageSpecPosAll, true);
%}
save("/Users/oliverbroadrick/Desktop/glitter-stuff/jan12/train_after_charMeasOptimization","results");

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Now... for this xf (optimized camera position for this characterization
% based on the "train" set of sparkle calibrations)... see if it
% "generalizes" to the "test" set of sparkle calibrations from another
% camera position... i.e., run the error function once for the "test" set
for index=1:numcases
    input = test(index);

    expdirTest{index} = input.expdir;
    curExpdir = expdirTest{index};
    impathTest{index} = [curExpdir input.impath];
    lightPosTest{index} = matfile([curExpdir input.lightPosFname]).lightPos;
    %fprintf('\n%s\n',input.name);
    skew = input.skew;
end

% do only the translation-estimating part of a sparkle calibration
warning('off','MATLAB:singularMatrix');
set(0,'DefaultFigureVisible','off');
other.inlierThreshold = 20;
% pass a setting to get a quick estimate rather than a fully-fine-tuned
% estimate of the camera position
other.quickEstimate = true;
for index=1:numcases
    [camPosEst, ~, mostInliersImageSpecPos, other] = estimateTglitter(impathTest{index}, ...
                                                lightPosTest{index}, ...
                                                getPoints(expdirTest{index}), ...
                                                expdirTest{index}, ...
                                                -1, ...
                                                skew, ...
                                                other);
    mostInliersLTest{index} = other.mostInliersL; % this extra return of the estimateTglitter function gives us the rays from light position to spec locations
    mostInliersSpecPosTest{index} = other.mostInliersSpecPos;
    mostInliersIdxsTest{index} = other.mostInliersIdxs;
    lightPosTest{index} = other.lightPos;
    mostInliersRoriginalTest{index} = other.mostInliersR;
    overallIdxTest{index} = other.overallIdx;
    mostInliersKminTest{index} = other.mostInliersKmin;
    mostInliersImageSpecPosTest{index} = mostInliersImageSpecPos;
    % get the relevant lighting means
    for ix=1:size(mostInliersSpecPosTest{index},1)
        mostInliersOverallIdxsTest{index}(ix) = overallIdxTest{index}(mostInliersIdxsTest{index}(ix), mostInliersKminTest{index}(ix));
    end
    mostInliersLightingMeansTest{index}(:,:) = lightingMeans(:,mostInliersOverallIdxsTest{index});
end

%% get results on the "test" set(s) to visualize/compare also
% Both cam position and monitor relative position measurements
[error,results] = computeError(xf(4),xf(5),xf(6),MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdirTest, ...
                              mostInliersLightingMeansTest, ...
                              mostInliersSpecPosTest, xf(1:3), lightPosTest, ...
                              test, mostInliersImageSpecPosTest, true);
%{
% Just monitor relative position measurements
[error,results] = computeError(xf(1),xf(2),xf(3),MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdirTest, ...
                              mostInliersLightingMeansTest, ...
                              mostInliersSpecPosTest, charCamPos, lightPosTest, ...
                              test, mostInliersImageSpecPosTest, true);
%}
save("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/test_after_charMeasOptimization","results");

[error,results] = computeError(xf(4),xf(5),xf(6),MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdirTest, ...
                              mostInliersLightingMeansTest, ...
                              mostInliersSpecPosTest, xf(1:3), lightPosTest, ...
                              test, mostInliersImageSpecPosTest, true);
%{
[error,results] = computeError(xf(1),xf(2),xf(3),MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdirTest, ...
                              mostInliersLightingMeansTest, ...
                              mostInliersSpecPosTest, charCamPos, lightPosTest, ...
                              test, mostInliersImageSpecPosTest, true);
%}
save("/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/test_after_charMeasOptimization","results");


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function [error, results] = computeError(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,...
                              GLIT_TO_MON_EDGES_Y,MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdir, ...
                              mostInliersLightingMeans, ...
                              mostInliersSpecPos, camPos, lightPos, ...
                              input, mostInliersImageSpecPos, getResults)
    
    charM = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,...
                              GLIT_TO_MON_EDGES_Y);
    % track sums of camera position estimates to compute average at end
    camPosEst = [0 0 0];
    knownCamPos = [0 0 0];
    numcases = 10;
    for index=1:numcases
        % recompute normals only of the sparkling specs
        mostInliersNewNormals = computeSomeNormals(mostInliersLightingMeans{index}, mostInliersSpecPos{index}, camPos, charM);
    
        % re-reflect the rays for the sparkling specs
        R = reflect(mostInliersSpecPos{index}, mostInliersNewNormals, lightPos{index});
        R = reshape(R,size(R,1),size(R,3));
        
        % compute the new camera position estimate according to these rays
        curCamPosEst = nearestPointManyLines(mostInliersSpecPos{index}, mostInliersSpecPos{index}+R);
        camPosEsts{index} = curCamPosEst;
        knownCamPoss{index} = matfile([expdir{index} '/' num2str(index) '/camPosSkew.mat']).camPos; 
        if getResults
            % also compute the intrinsics
            results(index,:) = [curCamPosEst ...
                        linearEstimateRKglitter(input(index).impath,...
                                                curCamPosEst, ...
                                                getPoints(expdir{index}), ...
                                                mostInliersSpecPos{index}, ...
                                                mostInliersImageSpecPos{index}, ...
                                                expdir{index}, ...
                                                input(index).skew)];
        end
    end

    % Find differences between each sparkle calibration and get them to
    % approach the single average of the checkerboard estimates
    checkerAvg = 0;
    for i=1:numcases
        checkerAvg = checkerAvg + knownCamPoss{i};
    end
    checkerAvg = checkerAvg / numcases;
    diffSum = 0;
    for i=1:numcases
        curDiff = norm(camPosEsts{i} - checkerAvg);
        diffSum = diffSum + curDiff;
    end

    % the overall error is the difference in the average camera position
    % estimates of each method.
    error = diffSum;
end

function [error, results] = computeErrorOriginal(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,...
                              GLIT_TO_MON_EDGES_Y,MON_WIDTH_MM,...
                              MON_HEIGHT_MM, chardir, expdir, ...
                              mostInliersLightingMeans, ...
                              mostInliersSpecPos, camPos, lightPos, ...
                              input, mostInliersImageSpecPos, getResults)
    
    charM = setMeasurements(GLIT_TO_MON_PLANES,GLIT_TO_MON_EDGES_X,...
                              GLIT_TO_MON_EDGES_Y);
    % track sums of camera position estimates to compute average at end
    camPosEst = [0 0 0];
    knownCamPos = [0 0 0];
    numcases = 10;
    for index=1:numcases
        % recompute normals only of the sparkling specs
        mostInliersNewNormals = computeSomeNormals(mostInliersLightingMeans{index}, mostInliersSpecPos{index}, camPos, charM);
    
        % re-reflect the rays for the sparkling specs
        R = reflect(mostInliersSpecPos{index}, mostInliersNewNormals, lightPos{index});
        R = reshape(R,size(R,1),size(R,3));
        
        % compute the new camera position estimate according to these rays
        curCamPosEst = nearestPointManyLines(mostInliersSpecPos{index}, mostInliersSpecPos{index}+R);
        camPosEst = camPosEst + curCamPosEst;
        knownCamPos = knownCamPos + matfile([expdir{index} '/' num2str(index) '/camPosSkew.mat']).camPos; 
        if getResults
            % also compute the intrinsics
            results(index,:) = [curCamPosEst ...
                        linearEstimateRKglitter(input(index).impath,...
                                                curCamPosEst, ...
                                                getPoints(expdir{index}), ...
                                                mostInliersSpecPos{index}, ...
                                                mostInliersImageSpecPos{index}, ...
                                                expdir{index}, ...
                                                input(index).skew)];
        end
    end

    % knownCamPos and camPosEst currently have sums across all numcases 
    % trials for each of sparkles and checkerboards. 
    % now we compute the average position estimate for each.
    avgKnownCamPos = knownCamPos ./ numcases;
    avgCamPosEst = camPosEst ./ numcases;

    % the overall error is the difference in the average camera position
    % estimates of each method.
    error = norm(avgKnownCamPos-avgCamPosEst);
end

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
