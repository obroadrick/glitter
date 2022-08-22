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

function e = errRK(fx, fy, s, w, h, r1, r2, r3, cx, cy, imageSpecs, worldSpecs,...
    imageFiducials, worldFiducials, T, plottingFigure)
    % fx and fy are the x and y direction focal lengths
    % s is the skew
    % w and h are width and height in pixels of the image
    % r1, r2, r2 are the rodrigues parameters of the candidate rotation
    % p is a list of the known image points
    % Pts is a corresponding list of the known world points

    % get intrinsics matrix
    %{ 
    disp('fx:');
    disp(fx);
    disp('fy:');
    disp(fy);
    disp('s:');
    disp(s);
    disp('r123');
    disp(r1);
    disp(r2);
    disp(r3);
    %}
            
    %K = [10^(3)*fx s w/2; 0 10^(3)*fy h/2; 0 0 1];
    K = [10^(3)*fx s cx; 0 10^(3)*fy cy; 0 0 1];

    % get rotation matrix from rodrigues parameters
    R = rodrigues(r1,r2,r3);
    disp(r1,r2,r3);
    disp('initial R and K interpreted in errRK');
    disp(R);
    disp(K);

    % estimate image spec coordinates for world spec points
    imageSpecEstimates = [];
    for ix=1:size(worldSpecs,1)
        worldSpec = reshape(worldSpecs(ix,:),3,1);
        imageSpecEstimates(ix,:) = K * (R * (worldSpec - T));
        imageSpecEstimates(ix,:) = imageSpecEstimates(ix,:) ./ imageSpecEstimates(ix,3);
    end
    
    % estimate image fiducial coordinates for world fiducial points
    imageFiducialEstimates = [];
    for ix=1:size(worldFiducials,1)
        %disp(size(worldFiducials));
        worldFiducial = reshape(worldFiducials(ix,:),3,1);
        imageFiducialEstimates(ix,:) = K * (R * (worldFiducial - T));
        imageFiducialEstimates(ix,:) = imageFiducialEstimates(ix,:) ./ imageFiducialEstimates(ix,3);
    end


    % draw the same way as done in the linear script to confirm that the
    % information made its way here without issues
    %{
    figure;
    plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;    
    title('AFTER PASSING TO ERRRK: the original image specs (green) and projected by M specs (red)');
    for ix=1:size(imageSpecs,1)
        projectedSpec = K*R * (worldSpecs(ix,:)' - T);
        projectedSpec = projectedSpec ./ projectedSpec(3);
        plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
    end
    disp('here we are');
    disp(R);
    disp(K);
    %}

    % compute error for specs as a sum of the square differences
    eSpecs = 0;
    %disp('current guesses');
    %disp('f');
    %disp(f);
    %disp('r1,2,3');
    %disp(r1);
    %disp(r2);
    %disp(r3);
    for ix=1:size(imageSpecs,1)
        %disp('specs: estimated point by this model from known/characterized specs');
        %disp(imageSpecEstimates(ix,1:2));
        %disp('specs: image points observed (of glitter specs)');
        %disp(imageSpecs(ix, :));
        %disp('specs: difference');
        %disp(imageSpecEstimates(ix,1:2) - imageSpecs(ix, :));
        %disp('specs: norm of difference');
        %disp(norm(imageSpecEstimates(ix,1:2) - imageSpecs(ix, :)));
        %disp('specs: norm of difference squared');
        %disp(norm(imageSpecEstimates(ix,1:2) - imageSpecs(ix, :))^2);
        eSpecs = eSpecs + norm(imageSpecEstimates(ix,1:2) - imageSpecs(ix, :));
    end
    %disp('specs: size imageSpecEstimates:');
    %disp(size(imageSpecEstimates));
    %disp('specs: size imageSpecs:');
    %disp(size(imageSpecs));
    % make the reported error the average(ish) difference
    eSpecs = sqrt(eSpecs / size(imageSpecEstimates,1));

    % compute error for fiducials as a sum of the square differences
    eFiducials = 0;
    %disp('current guesses');
    %disp('f');
    %disp(f);
    %disp('r1,2,3');
    %disp(r1);
    %disp(r2);
    %disp(r3);
    for ix=1:size(imageFiducialEstimates,1)
        %disp(size(imageFiducialEstimates));
        %disp('fiducials: estimated point by this model from known/measured fiducials');
        %disp(imageFiducialEstimates(ix,1:2));
        %disp('fiducials: image points observed (of fiducials)');
        %disp(imageFiducials(ix, :));
        %disp('fiducials: difference');
        %disp(imageFiducialEstimates(ix,1:2) - imageFiducials(ix, :));
        %disp('fiducials: norm of difference');
        %disp(norm(imageFiducialEstimates(ix,1:2) - imageFiducials(ix, :)));
        %disp('fiducials: norm of difference squared');
        %disp(norm(imageFiducialEstimates(ix,1:2) - imageSpecs(ix, :))^2);
        eFiducials = eFiducials + norm(imageFiducialEstimates(ix,1:2) - imageFiducials(ix, :))^1;
    end
    %disp('fiducials: size imageFiducialEstimates:');
    %disp(size(imageFiducialEstimates));
    %disp('fiducials: size imageFiducials:');
    %disp(size(imageFiducials));
    % make the reported error the average(ish) difference
    eFiducials = eFiducials / size(imageFiducialEstimates,1);

    % plot the fiducial markers both known and estimated
    figure(plottingFigure);
    strs = ["1","2","3","4"];
    hold off;
    plot([imageFiducials(:,1); imageFiducials(1,1)], [imageFiducials(:,2); imageFiducials(1,2)],'Color','green');
    hold on;
    text(imageFiducials(:,1), imageFiducials(:,2), strs);
    text(imageFiducialEstimates(:,1), imageFiducialEstimates(:,2), strs);
    draw = max(max(imageFiducialEstimates,2),1) < 8000 & min(min(imageFiducialEstimates,2),1) > 0;
    if draw
        plot([imageFiducialEstimates(:,1); imageFiducialEstimates(1,1)], [imageFiducialEstimates(:,2); imageFiducialEstimates(1,2)],'Color','red');
    end
    % plot the specs both known and estimated
    plot([imageSpecs(:,1); imageSpecs(1,1)], [imageSpecs(:,2); imageSpecs(1,2)],'gx');
    %text(imageFiducials(:,1), imageFiducials(:,2), strs);
    %text(imageFiducialEstimates(:,1), imageFiducialEstimates(:,2), strs);
    if draw
        plot([imageSpecEstimates(:,1); imageSpecEstimates(1,1)], [imageSpecEstimates(:,2); imageSpecEstimates(1,2)], 'r+');
    end
    drawnow;

    % give a soft penalty for guesses with very different fx and fy's
    % to help steer the optimization a bit towards reasonable answers
    fxfyPenalty = (fx - fy)^2;

    e = eFiducials + eSpecs + fxfyPenalty;% + eSpecs;
end