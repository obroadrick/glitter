figure;
tiledlayout(3,4,"TileSpacing",'tight','Padding','tight');
for index=1:10
expir = ['/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/' num2str(index) '/'];
expdir = expir;
skew = true;
P = matfile('/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/paths.mat').P;
if ~skew
    camParams = matfile([expir 'camParams.mat']).camParams;
    camParamsErrors = matfile([expir 'camParamsErrors.mat']).camParamsErrors;
else
    camParams = matfile([expir 'camParamsSkew.mat']).camParams;
    camParamsErrors = matfile([expir 'camParamsErrorsSkew.mat']).camParamsErrors;
end
impath = [expir 'A_onGlitterPlane' num2str(index) '.JPG'];
imPath = impath;
if ~isfile([expdir '16pts.mat'])
    setenv('PATH', [getenv('PATH') ':/opt/homebrew/bin/python3.10:/opt/homebrew/bin:/opt/homebrew/sbin']);
    cmd = sprintf('python3.10 /Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/16ptsfinder.py "%s" "%s"', impath, [expdir '16pts.mat']);
    system(cmd);
    disp('Found new points');
end
nexttile;
pin = loadPoints([expir '16Pts.mat'], true);
imagesc(imread(impath));
hold on;
plot(pin(:,1),pin(:,2),'CX','markersize',10,'linewidth',2);
end