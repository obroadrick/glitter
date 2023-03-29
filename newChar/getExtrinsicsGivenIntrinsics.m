% get checkerboard extrinsics given checkerboard intrinsics and a new
% homography image

expdirOverall = '/Users/oliverbroadrick/Desktop/glitter-stuff/c2/';
skew = true;

numPositions = 6;
for i=1:numPositions
    % get this experiment's directory
    expdir = [expdirOverall num2str(i) '/'];
    imname = 'homography.JPG';

    % get intrinsics from previous calibration
    if skew
        camParams = matfile([expdir 'camParamsSkew.mat']).camParams;
    else
        camParams = matfile([expdir 'camParams.mat']).camParams;
    end
    
    % detect points in new image
    other.perImageNaming = false;
    % use the single overall homography image
    [~, imagePoints, worldPoints] = getCheckerboardHomography(expdirOverall, imname, other);
    
    % get extrinsicss
    worldPoints3d = [worldPoints zeros(size(worldPoints,1),1)]; 
    [camRot,camPos] = estimateWorldCameraPose(imagePoints,worldPoints3d,camParams);
    
    % save results
    if skew
        save([expdir 'camPosSkew.mat'], "camPos");
        save([expdir 'camRotSkew.mat'], "camRot");
    else
        save([expdir 'camPos.mat'], "camPos");
        save([expdir 'camRot.mat'], "camRot");
    end
end