myDir = '/Users/oliverbroadrick/Desktop/glitter-stuff/iphoneXR2/checkerboardsFixed/'; %gets directory
myFiles = dir(fullfile(myDir,'*.JPG')); %gets all wav files in struct
for k = 1:length(myFiles)
  baseFileName = myFiles(k).name;
  fullFileName = fullfile(myDir, baseFileName);
  fprintf(1, 'Now reading %s\n', fullFileName);
  im = imread(fullFileName);
  im = imrotate(im, 180);
  imwrite(im, fullFileName);
end