% Get the camera position in the one and only coordinate system :-)
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/mar3-characterization/';
camParams = matfile([chardir 'camParamsSkew.mat']).camParams;
translationVector = camParams.TranslationVectors(1,:);
Rvec = camParams.RotationVectors(1,:);
Rc = rotationVectorToMatrix(Rvec);
[~, camPos] = extrinsicsToCameraPose(Rc, translationVector);
save([chardir 'camPosSkew.mat'], "camPos");
camRot = Rc;
save([chardir 'camRotSkew.mat'], "camRot");