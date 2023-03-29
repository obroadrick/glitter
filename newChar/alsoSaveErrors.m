expdir = '/Users/oliverbroadrick/Desktop/glitter-stuff/ICCV_camPos1/';
numSubsets = 6;
for i=1:numSubsets
    
    dirPath = [expdir num2str(i)];
    chardir = [dirPath '/'];
    camParams = matfile([chardir 'camParamsSkew.mat']).camParams;
    camParamsErrors = matfile([chardir 'camParamsErrorsSkew.mat']).camParamsErrors;
    ApositionIndex = 1;size(camParams.TranslationVectors,1);
    translationVector = camParams.TranslationVectors(ApositionIndex,:);
    camPosErr = camParamsErrors.ExtrinsicsErrors.TranslationVectorsError(ApositionIndex,:);
    camRotErr = camParamsErrors.ExtrinsicsErrors.RotationVectorsError(ApositionIndex,:);
    Rvec = camParams.RotationVectors(ApositionIndex,:);
    Rc = rotationVectorToMatrix(Rvec);
    [~, camPos] = extrinsicsToCameraPose(Rc, translationVector);
    camRot = Rc';


    if skew
        save([chardir 'camPosSkew.mat'], "camPos");
        save([chardir 'camRotSkew.mat'], "camRot");
        save([chardir 'camPosErrSkew.mat'], "camPosErr");
        save([chardir 'camRotErrSkew.mat'], "camRotErr");
    else
        save([chardir 'camPos.mat'], "camPos");
        save([chardir 'camRot.mat'], "camRot");
        save([chardir 'camPosErr.mat'], "camPosErr");
        save([chardir 'camRotErr.mat'], "camRotErr");
    end
end