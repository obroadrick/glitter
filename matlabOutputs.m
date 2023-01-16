
skew = true;
for index = 1:10
%expdir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/testingMatlab/odds/'];
%expdir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/jan12data/' num2str(index) '/'];
expdir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(index) '/'];


%% get checkerboard outputs for comparison
if skew
    camParams = matfile([expdir 'camParamsSkew']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrorsSkew']).camParamsErrors;
    camPos = matfile([expdir 'camPosSkew']).camPos;
    camRot = matfile([expdir 'camRotSkew']).camRot;
    camPosErr = matfile([expdir 'camPosErrSkew']).camPosErr;
    camRotErr = matfile([expdir 'camRotErrSkew']).camRotErr;
else
    camParams = matfile([expdir 'camParams']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrors']).camParamsErrors;
    camPos = matfile([expdir 'camPos']).camPos;
    camRot = matfile([expdir 'camRot']).camRot;
    camPosErr = matfile([expdir 'camPosErr']).camPosErr;
    camRotErr = matfile([expdir 'camRotErr']).camRotErr;
end
fprintf('%d images used\n', camParams.NumPatterns)
omega = undoRodrigues(camRot);
fx = camParams.Intrinsics.FocalLength(1);
fy = camParams.Intrinsics.FocalLength(2);
cx = camParams.Intrinsics.PrincipalPoint(1);
cy = camParams.Intrinsics.PrincipalPoint(2);
s = camParams.Intrinsics.Skew;
rotAndIntrinsicsCheckerboards = [camPos omega(1) omega(2) omega(3) fx fy cx cy s];

allData(index,:) = rotAndIntrinsicsCheckerboards';

columnNames = ["Tx","Ty","Tz","omega1","omega2","omega3","fx","fy","cx","cy","s"];
fprintf(['%15s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n'], '', columnNames);
printRow('Checker est.', rotAndIntrinsicsCheckerboards);
% also get error estimates
fxe = camParamsErrors.IntrinsicsErrors.FocalLengthError(1);
fye = camParamsErrors.IntrinsicsErrors.FocalLengthError(2);
cxe = camParamsErrors.IntrinsicsErrors.PrincipalPointError(1);
cye = camParamsErrors.IntrinsicsErrors.PrincipalPointError(2);
se = camParamsErrors.IntrinsicsErrors.SkewError;
rotAndIntrinsicsCheckerboardsErrors = [camPosErr camRotErr fxe fye cxe cye se];
printRow('Given std.', rotAndIntrinsicsCheckerboardsErrors);
end

for i=1:11
    ranges(i) = max(allData(:,i)) - min(allData(:,i));
    stds(i) = std(allData(:,i));
end
fprintf('\n');
columnNames = ["Tx","Ty","Tz","omega1","omega2","omega3","fx","fy","cx","cy","s"];
fprintf(['%15s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n'], '', columnNames);
printRow('range:',ranges);
printRow('std:',stds);
