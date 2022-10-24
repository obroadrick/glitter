% slightly annoyingly, the camera calibration app will not save the camera
% calibration outputs (parameter estimates, error estimates) automatically
% but instead will allow you to output them to your matlab workspace... so
% this script takes those (by their default names) and then saves them to
% files named by the current date

% rename the matlab defaults to what i call them
camParams = cameraParams;
camParamsErrors = estimationErrors;

% save them
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
save([P.data 'camParams_' datestr(now, 'mm_dd_yyyy')], "camParams");
save([P.data 'camParamsErrors_' datestr(now, 'mm_dd_yyyy')], "camParamsErrors");

% save in the experiment/characterization directory
%chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/aug31characterization/';
%chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/sep19characterization(new-2)/';
chardir = '/Users/oliverbroadrick/Desktop/glitter-stuff/oct17characterization/';
save([chardir 'camParams'], "camParams");
save([chardir 'camParamsErrors'], "camParamsErrors");