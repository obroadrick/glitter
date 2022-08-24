% temporary testing to see that the reprojection of the "known" camera
% calibration is good:

% get the "known" camera parameters (both intrinsic and extrinsic)
% from matlab checkerboard calibration
%{ 
% from P path struct
camParams = matfile(P.camParams).camParams;
camParamsErrors = matfile(P.camParamsErrors).camParamsErrors;
camPos = matfile(P.camPos).camPos;
camRot = matfile(P.camRot).camRot;
%}
% from literals
%{
camParams = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/camParams_08_22_2022').camParams;
camParamsErrors = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/camParamsErrors_08_22_2022').camParamsErrors;
camPos = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/camPos_08_23_2022').camPos;
camRot = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/camRot_08_23_2022').camRot;
%}
% get these into K and R as we use them
K = camParams.IntrinsicMatrix';
% temporarily using what is in workspace for R and T for testing purposes.
%R = camRot;
%T = camPos;
% now plot the projection according to these parameters of the known world
% points and compare to the known corresponding image points
figure;
plot(imageSpecs(:,1),imageSpecs(:,2),'gx');
hold on;
title('TEST OF CHECKERBOARD: the original image specs (in green) and projected by K,R (in red)');
for ix=1:size(imageSpecs,1)
    projectedSpec = K * R * (worldSpecs(ix,:)' - T);
    projectedSpec = projectedSpec ./ projectedSpec(3);
    plot(projectedSpec(1),projectedSpec(2),'r+');hold on;
end


