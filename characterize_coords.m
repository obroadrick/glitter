p = '/Users/oliverbroadrick/Desktop/glitter-stuff/glitter-repo/data/';
means = matfile([p 'lightingmeans_2022_06_01.mat']).means;

%% spec to lightsource vectors in glitter coords
% at this point, means(1) is a list of x positions(in index units) of light that sparkled the centroids
% and means(2) is a list of y positions otherwise the same
% we want to get these to 3D coordinates in the glitter coordinates system
% let's get the first one (use mm)
GLIT_TO_MON_PLANES = 508;%MEASURED
GLIT_TO_MON_EDGES_X = 194;%MEASURED
GLIT_TO_MON_EDGES_Y = 71;%MEASURED
MON_WIDTH_MM = 670;%MEASURED
MON_HEIGHT_MM = 379;%MEASURED
MON_WIDTH_PXS = 3840;
MON_HEIGHT_PXS = 2160;
PX2MM_X = MON_WIDTH_MM / MON_WIDTH_PXS; 
PX2MM_Y = MON_HEIGHT_MM / MON_HEIGHT_PXS; 
INDEX_TO_PX = 15; % from Addy (confirmed) 
FIRST_INDEX_PX_X = 20; % from Addy (confirmed)
FIRST_INDEX_X = FIRST_INDEX_PX_X * PX2MM_X;
FIRST_INDEX_PX_Y = 20;
FIRST_INDEX_Y = FIRST_INDEX_PX_Y * PX2MM_Y;

% map the gaussian means to lighting positions
x = -GLIT_TO_MON_EDGES_X + MON_WIDTH_MM - (FIRST_INDEX_X + PX2MM_X * means(1,1)); 
y = -GLIT_TO_MON_EDGES_Y + MON_HEIGHT_MM - (FIRST_INDEX_Y + PX2MM_Y * means(1,2)); 
z = GLIT_TO_MON_PLANES;
frstCntrsLightngPos = [x;y;z];
%first glitter spec in glitter coord sys
h = 1; % homography from Addy
frstSpecPos = [h*C(1,1); h*C(1,2); 0];
%    ## find vector from light to spec ##
frstLightToSpec = -frstCntrsLightngPos - frstSpecPos;
% normalize
frstLightToSpec = frstLightToSpec / norm(frstLightToSpec);
disp(frstSpecPos);disp(frstCntrsLightngPos);disp(frstLightToSpec);


