function P = savePaths()
    % Saves relevant file paths for the project (data, images, etc)
    P.data = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
    P.checkerboardIms = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_10_2022/';    
    P.onGlitterPlane = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_10_2022/onglitterplane.JPG';
    P.imageCentroids = [P.data 'image_centroids_06_14_2022.mat'];
    P.canonicalCentroids = [P.data 'canonical_centroids_06_14_2022.mat'];
    P.tform = [P.data 'transform.mat'];
    P.camParams = [P.data 'camParams'];
    P.measurements = [P.data 'measurements.mat'];
    P.camPos = [P.data 'camera_in_glitter_coords_06_14_2022.mat'];
    P.leftRightSweep = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/left_right_captures/';
    P.upDownSweep = '/Users/oliverbroadrick/Desktop/glitter-stuff/new_captures/up_down_captures/';
    P.specNormals = [P.data 'spec_normals_06_14_2022.mat'];
    P.means = [P.data 'lightingmeans_06_14_2022.mat'];
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