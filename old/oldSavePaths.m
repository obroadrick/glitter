function P = savePaths()
    % Saves relevant file paths for the project (data, images, etc)
    P.data = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
    P.checkerboardIms = '/Users/oliverbroadrick/Desktop/glitter-stuff/july_characterization/checkerboards/';
    P.onGlitterPlane = '/Users/oliverbroadrick/Desktop/glitter-stuff/july_characterization/checkerboards/onGlitterPlane.JPG';
    P.imageCentroids = [P.data 'image_centroids_07_12_2022.mat'];
    P.canonicalCentroids = [P.data 'canonical_centroids_07_12_2022.mat'];
    P.tform = [P.data 'transform.mat'];
    P.camParams = [P.data 'camParams_07_12_2022'];
    P.measurements = [P.data 'measurements.mat'];
    P.camPos = [P.data 'camPos_07_12_2022.mat'];
    P.leftRightSweep = '/Users/oliverbroadrick/Desktop/glitter-stuff/july_characterization/2022-07-10T13,56,58calib0.0-Glitter/';
    P.upDownSweep = '/Users/oliverbroadrick/Desktop/glitter-stuff/july_characterization/2022-07-10T13,33,01calib-h429.0-Glitter/';
    P.specNormals = [P.data 'spec_normals_07_12_2022.mat'];
    P.means = [P.data 'lightingmeans_07_12_2022.mat'];
    P.characterizationTest = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/';
    P.extraOnGlitterPlane = '/Users/oliverbroadrick/Desktop/slant1.JPG';
    P.maxImage = [P.data 'maxImage.jpg'];
    P.maxBrightness = [P.data 'maxBrightness.mat'];
    % save paths 
    save([P.data 'paths'], "P");
end
% this could be updated to take as input a desired filename/item
% and then have it automatically find the most recent version of 
% such file/item so that I don't manually update the dates in these
% paths...