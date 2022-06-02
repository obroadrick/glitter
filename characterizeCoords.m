clear;
datap = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
means = matfile([datap 'lightingmeans_2022_06_01.mat']).means;
C = matfile([datap 'centroids_2022_06_01.mat']).C;
M = matfile([datap 'measurements.mat']).M;

%% spec to lightsource vectors in glitter coords
% first: just get the first such vector to see if it
% is reasonable

% map the gaussian means to lighting positions
% note that this can (and should and will) be recast
% as matrix operations and done for all of the points
% at once...
x = -M.GLIT_TO_MON_EDGES_X + M.MON_WIDTH_MM - (M.FIRST_INDEX_X + M.PX2MM_X * (means(1,1)-1)); 
y = -M.GLIT_TO_MON_EDGES_Y + M.MON_HEIGHT_MM - (M.FIRST_INDEX_Y + M.PX2MM_Y * (means(1,2)-1)); 
z = M.GLIT_TO_MON_PLANES;
lightPos = [x;y;z];
% first glitter spec in glitter coord sys
H = 1; % homography from Addy TODO
% easiest to wrap homography application and conversion
% back from homogeneous coordinates in a little function 
% probably? do so when we get homography.. for now x1
specPos = [H*C(1,1); H*C(1,2); 0];
% vector from spec to light
frstLightToSpec = lightPos + specPos;
% normalize
frstLightToSpec = frstLightToSpec / norm(frstLightToSpec);

% draw the glitter rig with this line showing
drawRig(M, reshape(lightPos, 1,1,3), reshape(specPos, 1,1,3));
