function P = savePaths()
    % Saves relevant file paths for the project (data, images, etc)
    P.data = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
    P.checkerboardIms = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_10_2022/';    
    P.onGlitterPlane = '/Users/oliverbroadrick/Desktop/glitter-stuff/checkerboards_06_10_2022/onglitterplane.JPG';
    P.imageCentroids = [P.data 'image_centroids_06_09_2022.mat'];
    P.canonicalCentroids = [P.data 'canonical_centroids_06_09_2022.mat'];
    P.tform = [P.data 'transform_06_08_2022'];
    P.measurements = [P.data 'measurements.mat'];
    P.camPos = [P.data 'camera_in_glitter_coords_06_08_2022.mat'];
    P.leftRightSweep = '/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures V (right - left)/';
    P.upDownSweep = '/Users/oliverbroadrick/Desktop/glitter-stuff/5-31 Captures H (top - bottom)/';
    P.specNormals = [P.data 'spec_normals_06_10_2022.mat'];
    P.means = [P.data 'lightingmeans_2022_06_02.mat'];
    P.characterizationTest = '/Users/oliverbroadrick/Desktop/glitter-stuff/characterization_test_images/';

    % save paths 
    save([P.data 'paths'], "P");
end
% this could be updated to take as input a desired filename/item
% and then have it automatically find the most recent version of 
% such file/item so that I don't manually update the dates in these
% paths...