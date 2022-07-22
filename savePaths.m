function P = savePaths()
    % raw images/inputs to the current characterization
    P.characterizationDirectory = '/Users/oliverbroadrick/Desktop/glitter-stuff/july19characterization/';
    P.leftRightSweep = [P.characterizationDirectory 'verBarSweep/'];
    P.upDownSweep = [P.characterizationDirectory 'horBarSweep/'];
    P.checkerboardIms = [P.characterizationDirectory 'checkerboards/'];
    P.onGlitterPlane = [P.checkerboardIms 'onGlitterPlane.JPG'];
    P.homographyImages = [P.characterizationDirectory 'homography/'];
    P.pointLightImages = [P.characterizationDirectory 'pointLightImages/'];
    P.characterizationPoints = [P.characterizationDirectory 'homographyPoints/16ptsJuly19-1.mat'];

    % specific image paths for testing and such
    %P.extraOnGlitterPlane = '/Users/oliverbroadrick/Desktop/slant1.JPG';
    %P.characterizationTest = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/';

    % data created by processing the images (surface normals, peak lighting
    % positions, camera parameters, etc)
    P.data = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
    P.measurements = [P.data 'measurements.mat'];
    
    P.tform = [P.data 'transform.mat'];

    P.camParams = [P.data 'camParams_07_20_2022'];
    P.camParamsErrors = [P.data 'camParamsErrors_07_20_2022'];
    P.camPos = [P.data 'camPos_07_20_2022.mat'];
    P.camRot = [P.data 'camRot_07_20_2022.mat'];

    P.maxImage = [P.data 'maxImageLeftRight_07_20_2022.jpg'];

    P.imageCentroids = [P.data 'image_centroids_07_22_2022.mat'];
    P.canonicalCentroids = [P.data 'canonical_centroids_07_22_2022.mat'];

    P.maxBrightness = [P.data 'maxBrightness_07_21_2022.mat'];

    P.means = [P.data 'lightingmeans_07_22_2022.mat'];

    P.specNormals = [P.data 'spec_normals_07_22_2022.mat'];

    % save paths 
    save([P.data 'paths'], "P");
end
% this could be updated to take as input a desired filename/item
% and then have it automatically find the most recent version of 
% such file/item so that I don't manually update the dates in these
% paths...