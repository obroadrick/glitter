function P = savePaths()
    % raw images/inputs to the current characterization
    P.characterizationDirectory = ['/Users/oliverbroadrick/Desktop/glitter-stuff/oct17characterization/'];
    P.leftRightSweep = [P.characterizationDirectory 'verBarSweep/'];
    P.upDownSweep = [P.characterizationDirectory 'horBarSweep/'];
    P.checkerboardIms = [P.characterizationDirectory 'checkerboards/'];
    P.onGlitterPlane = [P.checkerboardIms 'onGlitterPlane.JPG'];
    P.homographyImages = [P.characterizationDirectory 'homography/'];
    P.pointLightImages = [P.characterizationDirectory 'pointLightImages/'];
    P.characterizationPoints = [P.characterizationDirectory '16pts.mat'];
    P.maxBrightness = [P.characterizationDirectory 'maxBrightness.mat'];
    P.maxImage = [P.characterizationDirectory 'maxImageLeftRight.jpg'];
    P.imageCentroids = [P.characterizationDirectory 'image_centroids.mat'];
    P.canonicalCentroids = [P.characterizationDirectory 'canonical_centroids.mat'];
    P.means = [P.characterizationDirectory 'lightingmeans.mat'];
    P.specNormals = [P.characterizationDirectory 'spec_normals.mat'];
    P.camParams = [P.characterizationDirectory 'camParams'];
    P.camParamsErrors = [P.characterizationDirectory 'camParamsErrors'];
    P.camPos = [P.characterizationDirectory 'camPos.mat'];
    P.camRot = [P.characterizationDirectory 'camRot.mat'];
    P.measurements = [P.characterizationDirectory 'measurements.mat'];

    % specific image paths for testing and such
    %P.extraOnGlitterPlane = '/Users/oliverbroadrick/Desktop/slant1.JPG';
    %P.characterizationTest = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/circles_on_monitor/';

    % data created by processing the images (surface normals, peak lighting
    % positions, camera parameters, etc)
    P.data = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
    
    %{
    P.camParams = [P.data 'camParams_08_18_2022'];
    P.camParamsErrors = [P.data 'camParamsErrors_08_18_2022'];
    P.camPos = [P.data 'camPos_08_18_2022.mat'];
    P.camRot = [P.data 'camRot_08_18_2022.mat'];
    %}


    % save paths 
    save([P.data 'paths'], "P");
end
% this could be updated to take as input a desired filename/item
% and then have it automatically find the most recent version of 
% such file/item so that I don't manually update the dates in these
% paths...