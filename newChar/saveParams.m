% slightly annoyingly, the camera calibration app will not save the camera
% calibration outputs (parameter estimates, error estimates) automatically
% but instead will allow you to output them to your matlab workspace... so
% this script takes those (by their default names) and then saves them to
% files named by the current date

% rename the matlab defaults to what i call them
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