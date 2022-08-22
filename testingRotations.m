% temporary testing to see that the reprojection of the "known" camera
% calibration is good:

% get the "known" camera parameters (both intrinsic and extrinsic)
% from matlab checkerboard calibration
camParams = matfile(P.camParams).camParams;
camParamsErrors = matfile(P.camParamsErrors).camParamsErrors;
camPos = matfile(P.camPos).camPos;
camRot = matfile(P.camRot).camRot;
% get these into K and R as we use them
K = camParams.IntrinsicMatrix';
R = camRot;

% now plot the projection according to these parameters of the known world
% points and compare to the known corresponding image points
figure;
plot(imageSpecs(:,1),imageSpecs(:,2),'gx');hold on;
title('TEST OF CHECKERBOARD: the original image specs (green) and projected by M specs (red)');
for ix=1:size(imageSpecs,1)
    projectedSpec = K * R * (worldSpecs(ix,:)' - T);
    projectedSpec = projectedSpec ./ projectedSpec(3);
    plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
end


