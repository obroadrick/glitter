

skew = true;
expdir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/testingMatlab/odds/'];


%% get checkerboard outputs for comparison
if skew
    camParams = matfile([expdir 'camParamsSkew']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrorsSkew']).camParamsErrors;
    camPos = matfile([expdir 'camPosSkew']).camPos;
    camRot = matfile([expdir 'camRotSkew']).camRot;
else
    camParams = matfile([expdir 'camParams']).camParams;
    camParamsErrors = matfile([expdir 'camParamsErrors']).camParamsErrors;
    camPos = matfile([expdir 'camPos']).camPos;
    camRot = matfile([expdir 'camRot']).camRot;
end
omega = undoRodrigues(camRot);
fx = camParams.Intrinsics.FocalLength(1);
fy = camParams.Intrinsics.FocalLength(2);
cx = camParams.Intrinsics.PrincipalPoint(1);
cy = camParams.Intrinsics.PrincipalPoint(2);
s = camParams.Intrinsics.Skew;
rotAndIntrinsicsCheckerboards = [camPos omega(1) omega(2) omega(3) fx fy cx cy s];
columnNames = ["Tx","Ty","Tz","omega1","omega2","omega3","fx","fy","cx","cy","s"];
fprintf(['%15s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n'], '', columnNames);
printRow('Checker est.', rotAndIntrinsicsCheckerboards);
% also get error estimates
fxe = camParamsErrors.IntrinsicsErrors.FocalLengthError(1);
fye = camParamsErrors.IntrinsicsErrors.FocalLengthError(2);
cxe = camParamsErrors.IntrinsicsErrors.PrincipalPointError(1);
cye = camParamsErrors.IntrinsicsErrors.PrincipalPointError(2);
se = camParamsErrors.IntrinsicsErrors.SkewError;
rotAndIntrinsicsCheckerboardsErrors = [-1 -1 -1 -1 -1 -1 fxe fye cxe cye se];
printRow('Checker err.', rotAndIntrinsicsCheckerboardsErrors);
