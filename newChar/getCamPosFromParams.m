% Get the camera position in the one and only coordinate system :-)

% first save params
camParams = cameraParams;
camParamsErrors = estimationErrors;
chardir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/ICCV_camPos1/'];
skew=true;
if skew
save([chardir 'camParamsSkew'], "camParams");
save([chardir 'camParamsErrorsSkew'], "camParamsErrors");
else
save([chardir 'camParams'], "camParams");
save([chardir 'camParamsErrors'], "camParamsErrors");
end

% then get campos and camrot from params
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/ICCV_camPos1/';
camParams = matfile([chardir 'camParamsSkew.mat']).camParams;
ApositionIndex = 1;%size(camParams.TranslationVectors,1);
translationVector = camParams.TranslationVectors(ApositionIndex,:);
Rvec = camParams.RotationVectors(ApositionIndex,:);
Rc = rotationVectorToMatrix(Rvec);
[~, camPos] = extrinsicsToCameraPose(Rc, translationVector);
camRot = Rc';
if skew
    save([chardir 'camPosSkew.mat'], "camPos");
    save([chardir 'camRotSkew.mat'], "camRot");
else
    save([chardir 'camPos.mat'], "camPos");
    save([chardir 'camRot.mat'], "camRot");
end