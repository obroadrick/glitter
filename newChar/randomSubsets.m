% read in all the checkerboard image file names
%dirPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/jan13/checkerboards-all/';
dirPath = '/Users/oliverbroadrick/Desktop/glitter-stuff/feb10/checkerboards/';
allFiles = dir(dirPath);
allFiles=allFiles(~ismember({allFiles.name},{'.','..'}));

% using a random permutation of the order of all filenames, divide them
% into numSubsets subsets
numSubsets = 10;
p = randperm(size(allFiles,1));
perList = floor(size(allFiles,1) / numSubsets);
for i = 1:numSubsets
    list(i,:) = allFiles(p((i-1)*perList+1:i*perList));
end

% for each subset, move all those images to a subdirectory
for i=1:numSubsets
    % make new subdirectory
    subdirPath = [dirPath num2str(i)];
    mkdir(subdirPath);

    % move each image in this sublist into the new subdirectory
    for j=1:perList
        movefile([list(i,j).folder '/' list(i,j).name], subdirPath);
    end
end



